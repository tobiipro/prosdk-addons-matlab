%% ScreenBasedCalibrationValidation
%
% Provides methods and properties for managing calibration validation for screen based eye trackers.
%
%   calib_validation = ScreenBasedCalibrationValidation(tracker)
%
classdef ScreenBasedCalibrationValidation < handle
    properties (SetAccess = immutable)
        SampleCount
        TimeOut
    end

    properties (Access = {?TestScreenBasedCalibrationValidation})
        EyeTracker
        DisplayArea
        InValidationMode
        CollectedPoints
        GetGazeData
        StopGazeData
    end

    methods
        function calib_validation = ScreenBasedCalibrationValidation(new_tracker, sample_count, time_out)
            msgID = 'ScreenBasedCalibrationValidation:WrongInput';

            if nargin == 1
               sample_count = 30;
            end
            if nargin < 3
               time_out = 1000;
            end

            if ~isa(new_tracker,'EyeTracker')
                msg = 'Input must be an object from EyeTracker class.';
                throw(MException(msgID, msg));
            end

            if ~ismember(Capabilities.CanDoScreenBasedCalibration,new_tracker.DeviceCapabilities)
                msg = 'Eye tracker is not capable of perform a screen based monocular calibration.';
                baseException = MException(msgID, msg);
                throw(baseException);
            end

            if sample_count < 10 || sample_count > 3000
                msg = 'Number of samples must be between 10 and 3000.';
                throw(MException(msgID, msg));
            end

            if time_out < 100 || time_out > 3000
                msg = '"Time out must be between 100 and 3000 ms.';
                throw(MException(msgID, msg));
            end

            calib_validation.SampleCount = sample_count;
            calib_validation.TimeOut = time_out;

            calib_validation.InValidationMode = false;

            calib_validation.CollectedPoints = [];

            calib_validation.EyeTracker = new_tracker;

            calib_validation.GetGazeData = @new_tracker.get_gaze_data;
            calib_validation.StopGazeData = @new_tracker.stop_gaze_data;

            calib_validation.DisplayArea = calib_validation.EyeTracker.get_display_area;
        end

        %% Enter Validation Mode
        % Enters the calibration validation mode. Also starts subscribing to gaze data from the eye tracker.
        %
        %   calib_validation.enter_validation_mode()
        %
        function enter_validation_mode(calib_validation)
            if calib_validation.InValidationMode
                msgID = 'ScreenBasedCalibrationValidation:AlreadyInValidationMode';
                msg = 'Already in validation mode';
                throw(MException(msgID, msg));
            end

            calib_validation.GetGazeData();

            calib_validation.CollectedPoints = [];

            calib_validation.InValidationMode = true;
        end

        %% Leave Validation Mode
        % Leaves the calibration validation mode, clears all collected data, and unsubscribes from the eye tracker.
        %
        %   calib_validation.leave_validation_mode()
        %
        function leave_validation_mode(calib_validation)
            if ~calib_validation.InValidationMode
                msgID = 'ScreenBasedCalibrationValidation:NotInValidationMode';
                msg = 'Not in validation mode';
                throw(MException(msgID, msg));
            end

            calib_validation.InValidationMode = false;

            calib_validation.CollectedPoints = [];

            calib_validation.StopGazeData();
        end

        %% Collect Data
        % Starts collecting data for a calibration validation point. The argument used is the point the user is assumed
        % to be looking at and is given in the active display area coordinate system.
        %
        % Parameters: target_point2D (x,y) of the calibration validation point.
        %
        %   calib.calib_validation.collect_data(target_point2D)
        %
        function collect_data(calib_validation, target_point2D)
            if ~calib_validation.InValidationMode
                msgID = 'ScreenBasedCalibrationValidation:NotInValidationMode';
                msg = 'Not in validation mode';
                throw(MException(msgID, msg));
            end

            if ~isnumeric(target_point2D)
                msgID = 'ScreenBasedCalibrationValidation:WrongInput';
                msg = 'Coordinates must be numeric';
                throw(MException(msgID, msg));
            end

            if numel(target_point2D) ~= 2
                msgID = 'ScreenBasedCalibrationValidation:WrongInput';
                msg = 'Coordinates must be a vector with two values.';
                throw(MException(msgID, msg));
            end

            calib_validation.GetGazeData();

            pause(calib_validation.TimeOut/1000);

            gaze_data = calib_validation.GetGazeData();

            target_point3D = calib_validation.NormalizedPoint2DToPoint3D(target_point2D, calib_validation.DisplayArea);

            valid_samples = GazeData.empty(calib_validation.SampleCount, 0);

            valid_index = 0;

            gaze_point_left = zeros(calib_validation.SampleCount, 3);
            gaze_point_right = zeros(calib_validation.SampleCount, 3);
            gaze_origin_left = zeros(calib_validation.SampleCount, 3);
            gaze_origin_right = zeros(calib_validation.SampleCount, 3);

            for i=1:numel(gaze_data)
                if gaze_data(i).LeftEye.GazePoint.Validity && gaze_data(i).RightEye.GazePoint.Validity
                    valid_index = valid_index + 1;
                    valid_samples(valid_index) = gaze_data(i);

                    gaze_point_left(valid_index,:) = gaze_data(i).LeftEye.GazePoint.InUserCoordinateSystem;
                    gaze_point_right(valid_index,:) = gaze_data(i).RightEye.GazePoint.InUserCoordinateSystem;
                    gaze_origin_left(valid_index,:) = gaze_data(i).LeftEye.GazeOrigin.InUserCoordinateSystem;
                    gaze_origin_right(valid_index,:) = gaze_data(i).RightEye.GazeOrigin.InUserCoordinateSystem;

                    if valid_index == calib_validation.SampleCount
                        break;
                    end
                end
            end


            if valid_index < calib_validation.SampleCount
                if valid_index == 0
                    valid_samples = [];
                end
                calib_validation.CollectedPoints = [calib_validation.CollectedPoints; CalibrationValidationPoint(target_point2D, -1, -1, -1, -1, -1, -1, true, valid_samples)];
                return;
            end

            gaze_point_average_left = mean(gaze_point_left);
            gaze_point_average_right = mean(gaze_point_right);
            gaze_origin_average_left = mean(gaze_origin_left);
            gaze_origin_average_right = mean(gaze_origin_right);

            direction_gaze_point_left = calib_validation.NormalizedDirection(gaze_origin_average_left, gaze_point_average_left);
            direction_target_left = calib_validation.NormalizedDirection(gaze_origin_average_left, target_point3D);

            accuracy_left_eye = round(calib_validation.Angle(direction_target_left, direction_gaze_point_left), 3);

            direction_gaze_point_right = calib_validation.NormalizedDirection(gaze_origin_average_right, gaze_point_average_right);
            direction_target_right = calib_validation.NormalizedDirection(gaze_origin_average_right, target_point3D);

            accuracy_right_eye = round(calib_validation.Angle(direction_target_right, direction_gaze_point_right), 3);

            variance_left = mean(calib_validation.Angle(calib_validation.NormalizedDirection(gaze_origin_left, gaze_point_left), calib_validation.NormalizedDirection(gaze_origin_left, gaze_point_average_left)).^2);
            variance_right = mean(calib_validation.Angle(calib_validation.NormalizedDirection(gaze_origin_right, gaze_point_right), calib_validation.NormalizedDirection(gaze_origin_right, gaze_point_average_right)).^2);

            if variance_left < 0
                variance_left = 0;
            end

            if variance_right < 0
                variance_right = 0;
            end

            precision_left_eye = round(sqrt(variance_left), 3);
            precision_right_eye = round(sqrt(variance_right), 3);

            precision_rms_left_eye = round(calib_validation.RMS(calib_validation.NormalizedDirection(gaze_origin_left, gaze_point_left)), 3);
            precision_rms_right_eye = round(calib_validation.RMS(calib_validation.NormalizedDirection(gaze_origin_right, gaze_point_right)), 3);

            calib_validation.CollectedPoints = [calib_validation.CollectedPoints; CalibrationValidationPoint(target_point2D, accuracy_left_eye, precision_left_eye, precision_rms_left_eye, accuracy_right_eye, precision_right_eye, precision_rms_right_eye, false, valid_samples)];
        end


        %% Discard Data
        % Clears the collected data for a specific calibration validation point.
        %
        % Parameters: target_point2D (x,y) of the calibration validation point.
        %
        %   calib_validation.discard_data(target_point2D)
        %
        function discard_data(calib_validation, target_point2D)
            if ~calib_validation.InValidationMode
                msgID = 'ScreenBasedCalibrationValidation:NotInValidationMode';
                msg = 'Not in validation mode';
                throw(MException(msgID, msg));
            end

            if ~isnumeric(target_point2D)
                msgID = 'ScreenBasedCalibrationValidation:WrongInput';
                msg = 'Coordinates must be numeric';
                throw(MException(msgID, msg));
            end

            if numel(target_point2D) ~= 2
                msgID = 'ScreenBasedCalibrationValidation:WrongInput';
                msg = 'Coordinates must be a vector with two values.';
                throw(MException(msgID, msg));
            end

            non_discarded_points = [];
            for i=1:numel(calib_validation.CollectedPoints)
                if ~all(calib_validation.CollectedPoints(i).Coordinates == target_point2D)
                    non_discarded_points = [non_discarded_points; calib_validation.CollectedPoints(i)]; %#ok<AGROW>
                end
            end

            if numel(calib_validation.CollectedPoints) == numel(non_discarded_points)
                msgID = 'ScreenBasedCalibrationValidation:DiscardNonCollectedPoint';
                msg = 'Trying to discard data for a point that has not been collected yet';
                throw(MException(msgID, msg));
            end
            calib_validation.CollectedPoints = non_discarded_points;
        end

        %% Compute
        % Uses the collected data and tries to compute accuracy and precision values for all points. If the calculation
        % is successful, the result is returned.
        % If there is insufficient data to compute the results for a certain point that CalibrationValidationPoint will
        % contain invalid data (NaN) for the results. Gaze data will still be untouched. If there is no valid data for
        % any point, the average results of CalibrationValidationResult will be invalid (NaN) as well.
        %
        % Returns: an instance of the class CalibrationResult.
        %
        %   calib_validation.compute()
        %
        function result = compute(calib_validation)
            precision_left_eye = 0;
            accuracy_left_eye = 0;
            precision_rms_left_eye = 0;
            precision_right_eye = 0;
            accuracy_right_eye = 0;
            precision_rms_right_eye = 0;

            non_timed_out_count = 0;

            for i=1:numel(calib_validation.CollectedPoints)
                if ~calib_validation.CollectedPoints(i).TimedOut
                    precision_left_eye = precision_left_eye + calib_validation.CollectedPoints(i).PrecisionLeftEye;
                    precision_right_eye = precision_right_eye + calib_validation.CollectedPoints(i).PrecisionRightEye;
                    accuracy_left_eye = accuracy_left_eye + calib_validation.CollectedPoints(i).AccuracyLeftEye;
                    accuracy_right_eye = accuracy_right_eye + calib_validation.CollectedPoints(i).AccuracyRightEye;
                    precision_rms_left_eye = precision_rms_left_eye + calib_validation.CollectedPoints(i).PrecisionRMSLeftEye;
                    precision_rms_right_eye = precision_rms_right_eye + calib_validation.CollectedPoints(i).PrecisionRMSRightEye;
                    non_timed_out_count = non_timed_out_count + 1;
                end

            end

            average_precision_left_eye = round(precision_left_eye / non_timed_out_count, 3);
            average_precision_right_eye = round(precision_right_eye / non_timed_out_count, 3);
            average_accuracy_left_eye = round(accuracy_left_eye / non_timed_out_count, 3);
            average_accuracy_right_eye = round(accuracy_right_eye / non_timed_out_count, 3);
            average_precision_rms_left_eye = round(precision_rms_left_eye / non_timed_out_count, 3);
            average_precision_rms_right_eye = round(precision_rms_right_eye / non_timed_out_count, 3);

            result = CalibrationValidationResult(calib_validation.CollectedPoints, average_accuracy_left_eye, average_precision_left_eye, average_precision_rms_left_eye, ...
                                                 average_accuracy_right_eye, average_precision_right_eye, average_precision_rms_right_eye);
        end
    end

    methods (Static = true, Hidden = true)
        function ret = NormalizedPoint2DToPoint3D(point2D, display_area)
            dx = (display_area.TopRight - display_area.TopLeft)*point2D(1);
            dy = (display_area.BottomLeft - display_area.TopLeft)*point2D(2);
            ret = reshape(display_area.TopLeft + dx + dy, 1, 3);
        end

        function ret = NormalizedDirection(start_point3D, end_point3D)
            ret = ScreenBasedCalibrationValidation.Normalize(bsxfun(@minus, end_point3D, start_point3D));
        end

        function ret = Normalize(point)
            if all(point == 0)
                ret = point;
            else
                ret = point./norm(point);
            end
        end

        function ret = Angle(points1, points2)
            if size(points1,1) > 1
                ret = zeros(size(points1,1),1);
                for i=1:size(points1,1)
                    ret(i) = atan2d(norm(cross(points1(i,:), points2(i,:))),dot(points1(i,:), points2(i,:)));
                end
            else
                ret = atan2d(norm(cross(points1, points2)),dot(points1, points2));
            end
        end

        function ret = RMS(point_array)
            ret = 0;

            if size(point_array,1) > 1
                for i=1:size(point_array,1)-1
                    ret = ret + ScreenBasedCalibrationValidation.Angle(point_array(i,:), point_array(i+1,:)) ^ 2;
                end
            end

            ret = sqrt(ret / (size(point_array,1)-1));
        end
    end
end


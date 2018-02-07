classdef ScreenBasedCalibrationValidation < handle
    properties (SetAccess = immutable)
        SampleCount
        TimeOut
         CollectedPoints
    end

    properties (Access = private)
        EyeTracker
        DisplayArea
        InValidationMode
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

            calib_validation.DisplayArea = calib_validation.EyeTracker.get_display_area;
        end

        function enter_validation_mode(calib_validation)
            if calib_validation.InValidationMode
                msgID = 'ScreenBasedCalibrationValidation:AlreadyInValidationMode';
                msg = 'Already in validation mode';
                throw(MException(msgID, msg));
            end

            calib_validation.EyeTracker.get_gaze_data();

            calib_validation.CollectedPoints = [];

            calib_validation.InValidationMode = true;

        end

        function leave_validation_mode(calib_validation)
            if ~calib_validation.InValidationMode
                msgID = 'ScreenBasedCalibrationValidation:NotInValidationMode';
                msg = 'Not in validation mode';
                throw(MException(msgID, msg));
            end

            calib_validation.InValidationMode = false;

            calib_validation.CollectedPoints = [];

            calib_validation.EyeTracker.stop_gaze_data();

        end

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

            calib_validation.EyeTracker.get_gaze_data();

            pause(calib_validation.TimeOut/1000);

            gaze_data = calib_validation.EyeTracker.get_gaze_data();


            target_point3D = NormalizedPoint2DToPoint3D(target_point2D, calib_validation.DisplayArea);

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
                calib_validation.CollectedPoints = [calib_validation.CollectedPoints; CalibrationValidationPoint(target_point2D, -1, -1, -1, -1, true, valid_samples)];
                return;
            end

            gaze_point_average_left = mean(gaze_point_left);
            gaze_point_average_right = mean(gaze_point_right);
            gaze_origin_average_left = mean(gaze_origin_left);
            gaze_origin_average_right = mean(gaze_origin_right);

            direction_gaze_point_left = NormalizedDirection(gaze_origin_average_left, gaze_point_average_left);
            direction_target_left = NormalizedDirection(gaze_origin_average_left, target_point3D);

            accuracy_left_eye = Angle(direction_target_left, direction_gaze_point_left);

            direction_gaze_point_right = NormalizedDirection(gaze_origin_average_right, gaze_point_average_right);
            direction_target_right = NormalizedDirection(gaze_origin_average_right, target_point3D);

            accuracy_right_eye = Angle(direction_target_right, direction_gaze_point_right);

            variance_left = mean(Angle(NormalizedDirection(gaze_origin_left, gaze_point_left), NormalizedDirection(gaze_origin_left, gaze_point_average_left)).*2);
            variance_right = mean(Angle(NormalizedDirection(gaze_origin_right, gaze_point_right), NormalizedDirection(gaze_origin_right, gaze_point_average_right)).*2);

            if variance_left < 0
                variance_left = 0;
            end

            if variance_right < 0
                variance_right = 0;
            end

            precision_left_eye = sqrt(variance_left);
            precision_right_eye = sqrt(variance_right);

            calib_validation.CollectedPoints = [calib_validation.CollectedPoints; CalibrationValidationPoint(target_point2D, accuracy_left_eye, precision_left_eye, accuracy_right_eye, precision_right_eye, false, valid_samples)];
        end

        function discard_data(calib_validation, target_point2D)
            non_discarded_points = [];
            for i=1:numel(calib_validation.CollectedPoints)
                if ~all(calib_validation.CollectedPoints(i).Coordinates == target_point2D)
                    non_discarded_points = [non_discarded_points; calib_validation.CollectedPoints(i)]; %#ok<AGROW>
                end
            end

            calib_validation.CollectedPoints = non_discarded_points;
        end

        function result = compute(calibration_validation)
            precision_left_eye = 0;
            accuracy_left_eye = 0;
            precision_right_eye = 0;
            accuracy_right_eye = 0;

            non_timed_out_count = 0;

            for i=1:numel(calib_validation.CollectedPoints)
                if ~calib_validation.CollectedPoints(i).TimedOut
                    precision_left_eye = precision_left_eye + calib_validation.PrecisionLeftEye;
                    precision_right_eye = precision_right_eye + calib_validation.PrecisionRightEye;
                    accuracy_left_eye = accuracy_left_eye + calib_validation.AccuracyLeftEye;
                    accuracy_right_eye = accuracy_right_eye + calib_validation.AccuracyRightEye;
                    non_timed_out_count = non_timed_out_count + 1;
                end

            end

            avarage_precision_left_eye = precision_left_eye / non_timed_out_count;
            avarage_precision_right_eye = precision_right_eye / non_timed_out_count;
            avarage_accuracy_left_eye = accuracy_left_eye / non_timed_out_count;
            avarage_accuracy_right_eye = accuracy_right_eye / non_timed_out_count;

            average_precision = (avarage_precision_left_eye + avarage_precision_right_eye) / 2;
            average_accuracy = (avarage_accuracy_left_eye + avarage_accuracy_right_eye) / 2;

            result = CalibrationValidationResult(calibration_validation.CollectedPoints, average_accuracy, average_precision);

        end

    end

end

function ret = NormalizedPoint2DToPoint3D(point2D, display_area)
    dx = (display_area.TopRight - display_area.TopLeft)*point2D(1);
    dy = (display_area.BottomLeft - display_area.TopLeft)*point2D(2);
    ret = reshape(display_area.TopLeft + dx + dy, 1, 3);
end

function ret = NormalizedDirection(start_point3D, end_point3D)
    ret = Normalize(start_point3D - end_point3D);
end

function ret = Normalize(point)
    ret = point./norm(point);
end

function ret = Angle(point1, point2)
    ret = atan2d(norm(cross(point1, point2)),dot(point1, point2));
end
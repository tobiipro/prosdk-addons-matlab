%% CalibrationValidationPoint
%
% Represents a collected point that goes into the calibration validation. It contains calculated values for accuracy
% and precision as well as the original gaze samples collected for the point.
%
%   result = CalibrationValidationPoint(coordinates,accuracy_left_eye, precision_left_eye, precision_rms_left_eye, ...
%                                       accuracy_right_eye, precision_right_eye, precision_rms_right_eye,...
%                                       timed_out, gaze_data)
%
classdef CalibrationValidationPoint
    properties (SetAccess = protected)
        %% Coordinates
        % The 2D coordinates of this point (in Active Display Coordinate System).
        %
        %   result.Coordinates
        %
        Coordinates

        %% AccuracyLeftEye
        % The accuracy in degrees for the left eye.
        %
        %   result.AccuracyLeftEye
        %
        AccuracyLeftEye

        %% PrecisionLeftEye
        % The precision (standard deviation) in degrees for the left eye.
        %
        %   result.PrecisionLeftEye
        %
        PrecisionLeftEye

        %% PrecisionRMSLeftEye
        % The precision (root mean square of sample-to-sample error) in degrees for the left eye.
        %
        %   result.PrecisionRMSLeftEye
        %
        PrecisionRMSLeftEye

        %% AccuracyRightEye
        % The accuracy in degrees for the right eye.
        %
        %   result.AccuracyRightEye
        %
        AccuracyRightEye

        %% PrecisionRightEye
        % The precision (standard deviation) in degrees for the right eye.
        %
        %   result.PrecisionRightEye
        %
        PrecisionRightEye

        %% PrecisionRMSRightEye
        % The precision (root mean square of sample-to-sample error) in degrees for the right eye.
        %
        %   result.PrecisionRMSRightEye
        %
        PrecisionRMSRightEye

        %% TimedOut
        % A boolean indicating if there was a timeout while collecting data for this point.
        %
        %   result.TimedOut
        %
        TimedOut

        %% GazeData
        % The gaze data samples collected for this point. These samples are the base for the calculated accuracy
        % and precision.
        %
        %   result.GazeData
        %
        GazeData
    end

    methods
        function result = CalibrationValidationPoint(coordinates,accuracy_left_eye, precision_left_eye, ...
                                                     precision_rms_left_eye, accuracy_right_eye, precision_right_eye, ...
                                                     precision_rms_right_eye, timed_out, gaze_data)
            result.Coordinates = coordinates;
            result.AccuracyLeftEye = accuracy_left_eye;
            result.PrecisionLeftEye = precision_left_eye;
            result.PrecisionRMSLeftEye = precision_rms_left_eye;
            result.AccuracyRightEye = accuracy_right_eye;
            result.PrecisionRightEye = precision_right_eye;
            result.PrecisionRMSRightEye = precision_rms_right_eye;
            result.TimedOut = timed_out;
            result.GazeData = gaze_data;
        end
    end
end
%% CalibrationValidationResult
%
% Provides methods and properties for managing calibration validation for screen based eye trackers.
%
%   result = CalibrationValidationResult(points, average_accuracy_left_eye, average_precision_left_eye, ...
%                                        average_precision_rms_left_eye, average_accuracy_right_eye, ...
%                                        average_precision_right_eye, average_precision_rms_right_eye)
%
classdef CalibrationValidationResult
    properties (SetAccess = protected)
        %% Points
        % The results of the calibration validation per point (same points as were collected).
        %
        %   result.Points
        %
        Points

        %% AverageAccuracyLeftEye
        % The accuracy in degrees averaged over all collected points for the left eye.
        %
        %   result.AverageAccuracyLeftEye
        %
        AverageAccuracyLeftEye

        %% AveragePrecisionLeftEye
        % The precision (standard deviation) in degrees averaged over all collected points for the left eye.
        %
        %   result.AveragePrecisionLeftEye
        %
        AveragePrecisionLeftEye

        %% AveragePrecisionRMSLeftEye
        % The precision (root mean square of sample-to-sample error) in degrees averaged over all collected points
        % for the left eye.
        %
        %   result.AveragePrecisionRMSLeftEye
        %
        AveragePrecisionRMSLeftEye

        %% AverageAccuracyRightEye
        % The accuracy in degrees averaged over all collected points for the right eye.
        %
        %   result.AverageAccuracyRightEye
        %
        AverageAccuracyRightEye

        %% AveragePrecisionRightEye
        % The precision (standard deviation) in degrees averaged over all collected points for the right eye.
        %
        %   result.AveragePrecisionRightEye
        %
        AveragePrecisionRightEye

        %% AveragePrecisionRMSRightEye
        % The precision (root mean square of sample-to-sample error) in degrees averaged over all collected
        % points for the right eye.
        %
        %   result.AveragePrecisionRMSRightEye
        %
        AveragePrecisionRMSRightEye
    end

    methods
        function result = CalibrationValidationResult(points, average_accuracy_left_eye, average_precision_left_eye, average_precision_rms_left_eye,...
                                                      average_accuracy_right_eye, average_precision_right_eye, average_precision_rms_right_eye)
            result.Points = points;
            result.AverageAccuracyLeftEye = average_accuracy_left_eye;
            result.AveragePrecisionLeftEye = average_precision_left_eye;
            result.AveragePrecisionRMSLeftEye = average_precision_rms_left_eye;
            result.AverageAccuracyRightEye = average_accuracy_right_eye;
            result.AveragePrecisionRightEye = average_precision_right_eye;
            result.AveragePrecisionRMSRightEye = average_precision_rms_right_eye;
        end
    end
end
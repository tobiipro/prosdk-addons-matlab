classdef CalibrationValidationResult
    properties (SetAccess = immutable)
        Points
        AverageAccuracyLeftEye
        AveragePrecisionLeftEye
        AveragePrecisionRMSLeftEye
        AverageAccuracyRightEye
        AveragePrecisionRightEye
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
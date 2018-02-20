classdef CalibrationValidationResult
    properties (SetAccess = immutable)
        Points
        AverageAccuracy
        AveragePrecision
        AveragePrecisionRMS
    end

    methods
        function result = CalibrationValidationResult(points, average_accuracy, average_precision, average_precision_rms)
            result.Points = points;
            result.AverageAccuracy = average_accuracy;
            result.AveragePrecision = average_precision;
            result.AveragePrecisionRMS = average_precision_rms;
        end
    end
end
classdef CalibrationValidationResult
    properties (SetAccess = immutable)
        Points
        AverageAccuracy
        AveragePrecision
    end

    methods
        function result = CalibrationValidationResult(points, average_accuracy, average_precision)
            result.Points = points;
            result.AverageAccuracy = average_accuracy;
            result.AveragePrecision = average_precision;
        end
    end
end
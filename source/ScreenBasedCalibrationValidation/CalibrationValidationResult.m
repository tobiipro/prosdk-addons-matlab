classdef CalibrationValidationResult
    properties (SetAccess = immutable)
        Points
        AverageAccuracy
        AveragePrecision
        AverageRMS
    end

    methods
        function result = CalibrationValidationResult(points, average_accuracy, average_precision, average_rms)
            result.Points = points;
            result.AverageAccuracy = average_accuracy;
            result.AveragePrecision = average_precision;
            result.AverageRMS = average_rms;
        end
    end
end
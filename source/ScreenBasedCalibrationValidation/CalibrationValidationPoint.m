classdef CalibrationValidationPoint
    properties (SetAccess = immutable)
        Coordinates
        AccuracyLeftEye
        PrecisionLeftEye
        AccuracyRightEye
        PrecisionRightEye
        TimedOut
        GazeData
    end

    methods
        function result = CalibrationValidationPoint(coordinates,accuracy_left_eye, precision_left_eye, ...
                                                     accuracy_right_eye, precision_right_eye, ...
                                                     timed_out, gaze_data)
            result.Coordinates = coordinates;
            result.AccuracyLeftEye = accuracy_left_eye;
            result.PrecisionLeftEye = precision_left_eye;
            result.AccuracyRightEye = accuracy_right_eye;
            result.PrecisionRightEye = precision_right_eye;
            result.TimedOut = timed_out;
            result.GazeData = gaze_data;
        end
    end
end
# prosdk-addons-matlab - Calibration validation

## What is it
Add-ons for the Tobii Pro SDK.

![alt text](https://www.tobiipro.com/imagevault/publishedmedia/6rkt3jb83qlottsfh1ts/Tobii-Pro-SDK-with-VR-3_1-banner.jpg)


The Tobii Pro SDK is available at: https://www.tobiipro.com/product-listing/tobii-pro-sdk/ <br />
Documentation to the API: http://developer.tobiipro.com/matlab.html


### Calibration Validation
* To use this feature it is necessary to have the SDK in the Matlab's path.
* Next just add to the Matlab's path the downloaded or cloned folder of the this repository.


Do not hesitate to contribute to this project and create issues if you find something that might be wrong or could be improved.

#### Example
Before starting a calibration validation, it is needed some setup with the desired tracker.
```matlab
Tobii = EyeTrackingOperations();

eyetracker_address = 'Replace the address of the desired tracker';

eyetracker = Tobii.get_eyetracker(eyetracker_address);
```
Now it is possible to create a calibration validation object with the eye tracker object previously created.
More information about this class and its methods can be found in the [ScreenBasedCalibrationValidation](./source/ScreenBasedCalibrationValidation/ScreenBasedCalibrationValidation.m) definition.
```matlab
sample_count = 30;
time_out_ms = 1000;

calib = ScreenBasedCalibrationValidation(eyetracker, sample_count, time_out_ms);
```
The next step is to enter validation mode. Note that this action will lead to the tracker to start collecting gaze data.
```matlab
calib.enter_validation_mode();
```

List the points that are to be used during the validation.
```matlab
points_to_collect = [[0.1,0.1];[0.1,0.9];[0.5,0.5];[0.9,0.1];[0.9,0.9]];
```

When collecting data a point should be presented on the screen in the appropriate position.
```matlab
for i=1:size(points_to_collect,1)
    calib.collect_data(points_to_collect(i,:));
end
```

Next just call the compute method to obtain the calibration validation result object.
```
calibration_result = calib.compute();
```

Now the calibration result should be available for inspection.
This can be done by accessing each individual property:
```matlab
calibration_result.Points
calibration_result.AverageAccuracyLeftEye
calibration_result.AveragePrecisionLeftEye
calibration_result.AveragePrecisionRMSLeftEye
calibration_result.AverageAccuracyRightEye
calibration_result.AveragePrecisionRightEye
calibration_result.AveragePrecisionRMSRightEye
```

More information about the calibration validation result can be found in the [CalibrationValidationResult](./source/ScreenBasedCalibrationValidation/CalibrationValidationResult.m) definition.

If the result is satisfactory then the only thing left to do is to leave validation mode.
This action will clear all the data collected in the current validation session and will stop the gaze data collection
from the tracker.
```matlab
calib.leave_validation_mode()
```

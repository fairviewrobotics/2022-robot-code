# High Goal Vision Docs
These programs were originally written in C++ and have been rewritten in Python.

#### Requirements

- Numpy
- OpenCV 3+

## HighGoal module
This module contains methods for getting target information such as angles and distance.
It uses the scoring methods from the __target_scoring__ module to select the most viable
targets, and the pipeline methods from the __pipeline__ module to find potential target contours
in images.

### get_distance_to_target ( _target_pitch: float_, _target_height: float_, _camera_angle: float_ ) &#8594; float
Get the forward distance from the camera to the target.
#### Params

- __target_pitch__: the observed vertical angle between the camera and the target in radians
- __target_height__: the height of the target relative to the camera
- __camera_angle__: the vertical angle of the camera relative to the ground

#### Returns
The distance to the target in the same units as target_height

### calc_angle ( _value: float_, _center_value: float_, _focal_length: float_ ) &#8594; float
Calculate angle from linear position on screen (in radians)

#### Params

- __value__: the value to calculate the angle to
- __center_value__: the value corresponding to the center of the screen
- __focal_length__: the focal length of the camera

#### Returns
The angle in radians to the value

### find_target_angles ( _config: dict_, _image: numpy.ndarray_) &#8594; Tuple[float, float]
Find the yaw and pitch of the best target within an image.

#### Params

- __config__: the configuration dictionary
- __image__: the image to search for targets within

#### Returns
A tuple containing the yaw and pitch in radians

### find_targets ( _config: dict_, _image: numpy.ndarray_, _score_thresh: float_, _image_size: float_, _size_relative_thresh: float_ ) &#8594; List[Tuple[np.ndarray, Rect]]
Return a list of target rectangles given an image and some scoring parameters.

#### Params

- __config__: the configuration parameters for the filtering and camera
- __image__: the image to locate targets within
- __score_thresh__: the minimum score a target must exceed to be returned
- __image_size__: the size of the image as its larger dimension
- __size_relative_thresh__: a proportion of image_size. See target_scoring.target_size_score

#### Returns
A list of targets as tuples of their contour and rectangle

### find_highest_target( _center_y: float_, _targets: List[Tuple[numpy.ndarray, Rect]]_ ) &#8594; Rect
Get the highest target from a list of targets.

#### Params
- __center_y__: the y value of the center of the image
- __targets__: a list of targets, each represented with a tuple containing its contour and bounding rect

#### Returns
A rectangle around the target containing the highest point (or None if no targets are passed)


## pipeline module
This module contains methods for finding potential target contours in an image.

### get_filtered_contours( _config: dict_, _source_image: numpy.ndarray_ ) &#8594; List[numpy.ndarray]
Get a list of contours in an image after applying an HSV filter and morphological 
transformations.

#### Params
- __config__: the configuration parameters for the filtering
- __source_image__: the image to find contours within

#### Returns
A list of arrays that describe contours as a collection of points on the image

### apply_hsv_filter( _config: dict_, _image: numpy.ndarray_ ) &#8594; numpy.ndarray
Get a bit mask from the image containing points that pass an HSV filter.

#### Params
- __config__: the configuration parameters for the filtering
- __image__: the image to filter

#### Returns
A new image containing the bit mask

### apply_morph( _config: dict_, _image: numpy.ndarray_ ) &#8594; numpy.ndarray
Apply morphological transformations to the image. This involves blurring the image and then sharpening it to fill
holes and reduce noise. Depending on configuration settings, this may also do an additional blurring (dilation) of
the image.

#### Params

- __config__: the configuration parameters for the filtering
- __image__: the image to apply morphological transformations to

#### Returns
The new, transformed image

### find_contours( _image: numpy.ndarray_ ) &#8594; List[numpy.array]
Find contours within a bitmask.

#### Params
- __image__: the image to find contours within

#### Returns
A list of contours sorted by decreasing size.


## target_scoring module
This module contains methods to score and select contours based on how likely they are to be viable
targets.

### area_coverage_score ( _rect: Rect_, _contour: numpy.ndarray_ ) &#8594; float
Return a score for a contour based on what percentage of rectangle's area the given contour covers. This score gives
a measurement of how rectangular the contour is. Since targets are very rectangular, this score helps determine
which contours are most likely to mark real targets.

#### Params

- __rect__: the minimum bounding box around the contour of the target
- __contour__: an array of points that make up the contour of the target

#### Returns
The score of a target based on its area coverage by its bounding box.
Uses the following equation: 1.0 - (1.0 - coverage) ^ scoring_exponent_constant

### aspect_ratio_score( _rect: Rect_ ) &#8594; float
Score a target based on its aspect ratio. Aspect ratios should be close to 5"/2" = 2.5. These measurements are based on
the dimensions of the retro-reflective tape on the upper hub.

#### Params
- __rect__: the minimum bounding box around the target

#### Returns
The score of a target based on its aspect ratio. Uses the following equation:
max(1.0 - (abs(aspect - 2.6)) ^ scoring_aspect_ratio_constant, 0.0)

### target_angle_score( _rect: Rect_ ) &#8594; float
Score a target rectangle based on its angle. Target angles are expected to be in [0, 40]U[140, 180] (i.e. within
40 degrees of horizontal).

#### Params

- __rect__: the minimum bounding box around the target

#### Returns
1.0 if this is the case and 0.0 otherwise.

### target_size_score( _rect: Rect_, _image_size: float_, _relative_size_thresh: float_ ) &#8594; float
Score a target based on its relative size in the image. If a target is too small based on the passed
threshold, then it receives a score of 0.0; otherwise, it receives a score of 1.0.

#### Params

- __rect__: the minimum bounding box around the target
- __image_size__: the size of the image as its larger dimension
- __relative_thresh__: a proportion of image_size for calculating the size threshold

#### Returns
The score of the target based on its size.

### check_target( _rect: Rect_, _contour: numpy.ndarray_, _score_thresh: float_, _image_size: float_, _size_relative_thresh_: float ) &#8594; bool
Check if a contour with a given bounding rectangle (rect) is a valid target based on a score threshold
(score_thresh).

#### Params

- __rect__: the minimum rotated rectangle around the contour
- __contour__: an array of points to be considered as a target
- __score_thresh__: the minimum scoring threshold for the contour to be considered a valid target
- __image_size__: the size of the image as its larger dimension
- __size_relative_thresh__: a proportion of image_size. See target_scoring.target_size_score

#### Returns
Whether the potential target exceeds the given threshold. This is based on whether
the sum of the aspect ratio score, the area coverage score, the target angle score, and double the target
size score exceed five times the score threshold.


## rectangle module
This module contains a class for representing rotated rectangles.

### Rect (class) ( _box_points: numpy.ndarray_ )

#### Fields

- __box_points__: an array containing the rectangle's corners (sorted clockwise, beginning at the bottom right)
- __center__: the center point of the rectangle
- __height__: the height of the rectangle based on box_points
- __width__: the width of the rectangle based on box_points.
- __area__: the area of the rectangle
- __angle__: the angle of the rectangle from horizontal. Rectangles where height > width are considered rotated an additional 90 degrees


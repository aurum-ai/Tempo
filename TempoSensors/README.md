# TempoSensors
`TempoSensors` includes the `TempoCamera` and `TempoLabels` modules and will be the home for any features using Unreal's rendering to simulate synthetic sensor data or ground truth labels.

## Scene Capture
`TempoSensors` uses the Unreal's `USceneCaptureComponent2D` to capture sensor data from different perspectives in the simulated scene.

`UTempoSceneCaptureComponent2D` expands `USceneCaptureComponent2D` in a few ways:
- Adds support for a dynamic pixel buffer type, so that sensors can change render target formats at runtime
- Adds logic to handle Tempo's different time modes (Real Time and Fixed Step), blocking to ensure all sensor measurements are sent in the frame they are captured in Fixed Step mode
- Adds virtual methods to allow derived, concrete sensor types to declare whether they have pending requests, and to implement the logic to decode scene captures into sensor measurements.

## Sensor Types
### TempoCamera
Currently, `TempoSensors` has only one sensor type: `TempoCamera`. However, `TempoCamera` can capture several different types of data: color images, semantic label images, depth images, and bounding boxes. `TempoCamera` uses several tricks to improve usability and performance:
- It can capture all three of the above image types in a single render pass, thanks to a custom 8-byte pixel buffer and a "reinterpret cast" from floating point to fixed point depth in the `M_CameraPostProcess_WithDepth` post-process material.
- It uses a custom stencil buffer to capture labels, but it can also use visually-imperceptible changes in a material's subsurface color to override the label on certain parts of an object (for example, to label lane lines differently from the road itself).

`TempoCamera` leverages `UTempoSceneCaptureComponent2D`'s ability to dynamically change render target format and pixel buffer type to enable or disable depth rendering as needed, since adding depth incurs a not-insignificant performance hit (depth is rendered either way, but copying it doubles the size of the pixel buffer). Semantic labels are effectively free (since there are no 3-byte pixel buffers), so `TempoCamera` has only two modes (depth and no-depth). Similarly, bounding box computation can be enabled/disabled as needed.

These features are automatically enabled when requested by a client and disabled when no longer needed. Semantic labels are always available since they have minimal performance impact.

## TempoLabels
`TempoLabels` includes the `UTempoActorLabeler` subsystem, which "labels" meshes in the scene by assigning values to their custom depth stencil on `BeginPlay` and whenever a component is created.

To specify which meshes should be labeled with which labels, the user must create a Label Table. The Label Table is a `DataTable` with `SemanticLabel` row type. Each row can specify Actor classes and/or Static Mesh types that should get certain label.

When a Static Mesh is specified, all components with that Static Mesh type will be labeled. When an Actor class is specified, all meshes on that actor will get the label type, **except** those that have an explicit Static Mesh label.

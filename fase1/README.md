# Vision → World: Image-to-World Transform for Base Localization

This document summarizes **how the pixel detections are mapped to world FRD coordinates** and how to configure the node.

## Coordinate Frames

- **World (FRD)**: X forward, Y right, Z down (right-handed; +yaw is clockwise from above).
- **Body (FRD)**: Same axis directions as World, attached to the drone.
- **Camera (OpenCV)**: +Z **forward** along the optical axis, +X to the **right** in the image, +Y **down** in the image.

Extrinsics are given **as the pose of the camera expressed in the Body frame**:

- Rotation $ R_{b\leftarrow c} $: **maps camera vectors into body** (FRD).
- Translation $ \mathbf{t}_{b\leftarrow c} = [t_x,t_y,t_z]^T $: camera origin written in body coordinates (meters).  
  In FRD, a **positive** $t_z$ means the camera is **below** the body origin.

The drone pose in world is:

- $ \mathbf{p}_w^b = [x, y, z]^T $ (meters, FRD; note $z>0$ means downwards)

- $ \text{rpy}_w^b = (\phi,\theta,\psi) $ roll/pitch/yaw, **about Body axes**, with
  $
    R_{w\leftarrow b} = R_z(\psi)\,R_y(\theta)\,R_x(\phi)
  $
  (right-handed; because $+Z$ points down, a positive yaw is clockwise when viewed from above.)

The camera pose in world is:

$
\begin{aligned}
R_{w\leftarrow c} &= R_{w\leftarrow b}\,R_{b\leftarrow c} \\
\mathbf{p}_w^c &= \mathbf{p}_w^b + R_{w\leftarrow b}\,\mathbf{t}_{b\leftarrow c}
\end{aligned}
$

## Intrinsics and Distortion (Published Image)

The image published by your node is **center-cropped to square and resized to 800×800**. The intrinsics $K$ and distortion $D$ used by the transform must match **this published stream**.

You have two ways to configure intrinsics:

1. **Direct** (recommended when you calibrate the 800×800 stream): set `fx, fy, cx, cy`.
2. **Derived**: give the raw camera resolution and `camera_horizontal_fov`. The node reproduces the crop-to-square and the resize to compute the equivalent $K$ for 800×800.

Distortion coefficients are $D = [k_1,k_2,p_1,p_2,k_3]$ **for the published stream**.

## From Pixel to World: Two Methods

Let the detection center be $(u,v)$ in pixels of the published image. The node first **undistorts** and converts to a normalized camera ray:

$
\begin{aligned}
\begin{bmatrix} x_n \\ y_n \end{bmatrix}
&= \text{undistortPoints}\big((u,v), K, D\big) \\
\hat{\mathbf{r}}_c &= \frac{1}{\sqrt{x_n^2 + y_n^2 + 1}}\,
\begin{bmatrix} x_n \\ y_n \\ 1 \end{bmatrix}
\end{aligned}
$

Then transform the ray to world:

$
\hat{\mathbf{r}}_w = R_{w\leftarrow c}\,\hat{\mathbf{r}}_c, \quad
\mathbf{p}_w^c = \mathbf{p}_w^b + R_{w\leftarrow b}\,\mathbf{t}_{b\leftarrow c}
$

### A) `getApproximateBase` (ray–plane intersection)

Assume the base lies on a **horizontal plane** $z = z_0$ (FRD, $z>0$ down). Intersect the ray with that plane:

$
t = \frac{z_0 - (\mathbf{p}_w^c)_z}{(\hat{\mathbf{r}}_w)_z}, \qquad
\mathbf{p}_w^{\text{base}} = \mathbf{p}_w^c + t\,\hat{\mathbf{r}}_w
$

Conditions/notes:
- Requires correct **extrinsics** (especially **pitch/roll**), otherwise the denominator may be near zero and produce large errors.
- Works best when the base really is at a known altitude (e.g., floor at $z=z_0$).
- Robust and fast; uses only the detection center.

### B) `getAccurateBase` (PnP on a 1×1 m square)

If the base is a square of known size $S=1\ \text{m}$ and the detector provides an **oriented rectangle** aligned with the square’s projected contour, we can estimate the **3D pose** using PnP:

1. Build the 4 image points from Detection2D: center $(c_x,c_y)$, size $(w,h)$, rotation $\theta$.  
   Convert to a rotated rectangle and **order corners** TL, TR, BR, BL.
2. Define object points (meters) on $z=0$:
   $
   \{(-\tfrac{S}{2},-\tfrac{S}{2},0), (\tfrac{S}{2},-\tfrac{S}{2},0), (\tfrac{S}{2},\tfrac{S}{2},0), (-\tfrac{S}{2},\tfrac{S}{2},0)\}
   $
3. Solve $ \{R_{c\leftarrow o},\,\mathbf{t}_{c\leftarrow o}\} = \text{solvePnP}(\text{objPts},\text{imgPts},K,D)$.
4. The square center in **camera** is $ \mathbf{t}_{c\leftarrow o} $. Convert to world:
   $
   \mathbf{p}_w^{\text{base}} = \mathbf{p}_w^c + R_{w\leftarrow c}\,\mathbf{t}_{c\leftarrow o}
   $
5. (Optional) If you want the result **exactly** on a plane $z=z_0$, project along the world ray from $\mathbf{p}_w^c$ to $z=z_0$.

Conditions/notes:
- Requires the rectangle to **match the square’s projection** reasonably well; poor detections degrade accuracy.
- Sensitive to intrinsics/distortion correctness, and to extrinsics (as always).
- Use `SOLVEPNP_IPPE_SQUARE` when available; it is tailored for planar squares.

## What about the 800×800 resize/crop?

The publisher **center-crops** the raw frame to a square and **resizes** to 800×800. The node models this so that the intrinsics match the published stream:

- If you **provide `fx,fy,cx,cy`**, they are **assumed for 800×800** directly.
- If you provide **raw size + HFOV**, the node:
  1) computes $f_x$ for the raw width and principal point at center;  
  2) shifts the principal point after the center crop;  
  3) scales to 800×800.

## Parameter Reference

### Image / camera
- `image_width`, `image_height` — Published image size (default 800, 800).
- `fx`, `fy`, `cx`, `cy` — **Preferred**: calibrated intrinsics for the published stream.
- `camera_horizontal_fov`, `camera_original_width`, `camera_original_height` — Alternative way to derive intrinsics if you don’t have `fx/fy/cx/cy`.
- `dist_k1..dist_k3`, `dist_p1`, `dist_p2` — Distortion on the published stream.

### Detection
- `bbox_is_normalized` — `true` if Detection2D centers/sizes are in $[0,1]$; `false` if in pixels.

### Extrinsics (camera **in Body/FRD**)
- `camera_roll`, `camera_pitch`, `camera_yaw` — Euler angles (rad) such that  
  $ R_{b\leftarrow c} = R_z(\text{yaw})R_y(\text{pitch})R_x(\text{roll}) $ maps **camera vectors → body**.
  - Example: If the camera is mounted perfectly down-looking and aligned so that **image +x matches body +Y** and **image +y matches body +X**, you can keep these near zero.  
  - If your physical mount differs, set these to the small angular offsets needed to align camera to body.
- `camera_tx`, `camera_ty`, `camera_tz` — **Camera origin** in meters expressed in Body frame. In FRD, positive `camera_tz` means the camera is **below** the FCU.

### Square model / planes
- `bbox_real_size` — Real side length of the base square (meters). Default: `1.0`.
- `mean_base_height` — Plane $z=z_0$ used by `getApproximateBase` (FRD `z`, positive down).
- `accurate_project_to_ground` — If `true`, project the PnP result to `accurate_ground_z`.
- `accurate_ground_z` — Plane $z$ used for the optional projection above.

## When does each method work best?

- **Approximate (ray–plane)**:
  - Base lies on a known plane (e.g., floor).
  - You want stable alignment behavior using the **detection center** only.
  - Robust to small bbox shape errors.
- **Accurate (PnP)**:
  - You detect a **1×1 m square** and the rectangle really outlines the square’s **projected** contour.
  - You have good intrinsics/distortion and reasonably accurate extrinsics.
  - Provides metric depth without assuming plane height (though you may still project to a plane for consistency).

## Common Pitfalls & Quick Checks

1. **Huge coordinates unless pitch=0** → Your `camera_roll/pitch/yaw` are likely defined in the **wrong direction**. Remember they map **camera→body**. If you have angles that map **body→camera**, transpose: set them to the negative (or swap order), or simply enter the *inverse* rotation.
2. **Everything mirrored** → `bbox_is_normalized` or `image_width/height` mismatch.
3. **Wrong scale** → Intrinsics don’t correspond to the 800×800 stream. Either calibrate that stream or provide raw size + FOV that the node can convert to 800×800.
4. **PnP unstable** → The rotated rectangle corners must correspond to the square’s actual projected corners. If your detector returns axis-aligned boxes, prefer the **approximate** method or switch to a corner/contour detector.

## Minimal Integration Pattern

- Call `getApproximateBase(drone_pos, drone_rpy, bbox, mean_base_height)` inside your FSM for alignment-on-plane behavior.
- For final approach or when you trust the oriented rectangle, call `getAccurateBase(...)`. Optionally set `accurate_project_to_ground: true` to keep z consistent with your controller.


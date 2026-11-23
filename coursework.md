# CG2026 (COMS30020) — Coursework Implementation Plan
## Ray-Traced Lighting & Advanced Rendering Topics  ·  Workbooks 07 & 08

> A study/implementation plan that walks through every task in **Workbook 07 (Lighting and
> Shading)** and **Workbook 08 (Coursework Advanced Topics)**, with the formula, a working C++
> code example using this project's conventions (GLM 0.9.7.2 + the `sdw` library), **where it
> goes in the `RenderProject` codebase**, and **how to test it**.
>
> The starter `RenderProject` ships with an empty `Renderer` and two stub renderers
> (`RedNoiseRenderer`, `ColourSpectrumRenderer`). By the time you reach this plan you should
> have already built the ray tracer in Workbook 06. Section 1 lists that prerequisite.

---

## 0. How to use this document

1. **Read section 1 first** and confirm each prerequisite already works. Don't start 07 until
   Workbook 06's `getClosestValidIntersection` + a basic ray-traced render of the Cornell Box
   already produces the reference image.
2. Each task below has the same shape: **Goal → Formula → Example code → Where it goes → Test**.
   Copy the code, get it compiling and rendering, *then* read the surrounding theory in the
   workbook's slides/audio.
3. **Keep the fast renderers.** Workbook 06 Task 7 gave you `1`/`2`/`3` keys to switch between
   *wireframe / rasterised / ray-traced*. Ray tracing is slow, so your workflow should be:
   **navigate the camera with the fast wireframe/rasteriser, park it where you want, then flip to
   the ray tracer to look at the lighting.** Treat that switch as a debugging tool, not a demo.
4. Every effect in 07/08 builds on the one before it. Don't skip the order — the later tasks
   (Phong, mirrors, refraction, DoF) reuse the lighting/shadow machinery from earlier ones.
5. Use the build rules deliberately:
   `make debug` to develop, `make diagnostic` to hunt a segfault, `make speedy` only to *view* a
   finished render (its `-Ofast -funsafe-math-optimizations` can produce "unusual but predictable"
   results — don't trust it while debugging).

---

## 1. Where you start — prerequisites from Workbooks 01–06

You need **all** of these already in place before Workbook 07. If any is missing, go back and
finish it first.

| Done in | What you must have | Used by |
|---|---|---|
| 04 T2–T3 | `loadGeometry()` parses `cornell-box.obj` (vertices `v`, faces `f`) into `std::vector<ModelTriangle> triangles` with a scale factor (~0.35); `loadMaterials()` parses the `.mtl` into the `palette` | everything |
| 04 T5 | `projectVertexOntoCanvasPoint(cameraPosition, focalLength, vertexPosition)` → `CanvasPoint` | rasteriser / camera rays |
| 04 T8–T9 | Rasterised render + `1/Z` depth buffer | the `2` renderer |
| 05 T2–T5 | Orbit/`lookAt` camera control; camera orientation as `glm::mat3` | moving the camera |
| 06 T2–T3 | `getClosestValidIntersection(cameraPosition, rayDirection, skipIndex)` using the inverse-`DEM` solve + `u/v/u+v ≤ 1` + `t > 0` validation, returning a `RayTriangleIntersection` | **the core of everything below** |
| 06 T5–T6 | Shadow test (can the point "see" the light?) + shadow-acne fix (offset the surface point along its normal by `ε`) | every lit render |
| 06 T7 | `1`/`2`/`3` renderer switching + per-frame event loop | your whole workflow |

**The one equation everything rests on** (06 T2). Given a ray origin `O` and unit direction
`d`, and triangle edges `e0 = v1 - v0`, `e1 = v2 - v0`:

```cpp
glm::vec3 e0 = t.vertices[1] - t.vertices[0];
glm::vec3 e1 = t.vertices[2] - t.vertices[0];
glm::vec3 sp = cameraPosition - t.vertices[0];
glm::mat3 DEM(-rayDirection, e0, e1);
glm::vec3 sol = glm::inverse(DEM) * sp;      // sol = (t, u, v)
// intersection point (either form):
glm::vec3 p1 = cameraPosition + sol.x * rayDirection;            // relative to camera
glm::vec3 p2 = t.vertices[0] + sol.y * e0 + sol.z * e1;          // relative to world origin
// barycentric weights of the three vertices:
float w0 = 1.0f - sol.y - sol.z;   // weight of vertex 0
float w1 = sol.y;                  // weight of vertex 1
float w2 = sol.z;                  // weight of vertex 2
```

Those `w0/w1/w2` weights are reused by **Gouraud, Phong and texture mapping** later — so keep the
raw `(t,u,v)` (or recompute the weights) when you extend the intersection.

---

## 2. Architecture you'll add

Add a new renderer class (per 06 T1) and a small shading toolkit:

```
src/
  RayTracedRenderer.h / .cpp      // new class, extends Renderer
  ColourSpectrumRenderer.*        // keep (a useful "visual debug" mode)
  RedNoiseRenderer.*              // keep
  MainApplication.cpp             // add: load the model, add the new key toggles
```

Suggested shape of `RayTracedRenderer.h`:

```cpp
class RayTracedRenderer : public Renderer {
    public:
        void draw(DrawingWindow &window) override;   // per-pixel: fire camera ray → shade hit
    private:
        // The single most important function: light a hit point on a triangle.
        uint32_t shadeIntersection(const RayTriangleIntersection &hit, int depth);

        // Per-task helpers (implement these one at a time as you progress).
        float proximity(const glm::vec3 &p) const;                       // 07 T2
        float diffuse(const ModelTriangle &t, const glm::vec3 &p) const; // 07 T3
        float specular(const ModelTriangle &t, const glm::vec3 &p) const;// 07 T4
        glm::vec3 barycentric(const RayTriangleIntersection &hit) const; // 07 T6 / 08 T2
        uint32_t sampleTexture(const ModelTriangle &t, const glm::vec3 &bary) const; // 08 T2
        bool canSeeLight(const glm::vec3 &from, const glm::vec3 &to, size_t skip) const; // 06 T5
};
```

Pack the final pixel the **same way** `RedNoiseRenderer` does — 8 bits per channel, A=0xFF:

```cpp
uint32_t packRGB(float r, float g, float b) {
    int R = std::min(255, std::max(0, (int)(r * 255.0f)));   // C++11: no std::clamp (that's C++17)
    int G = std::min(255, std::max(0, (int)(g * 255.0f)));
    int B = std::min(255, std::max(0, (int)(b * 255.0f)));
    return (255u << 24) | (R << 16) | (G << 8) | B;   // A<<24 | R<<16 | G<<8 | B
}
```

`ModelTriangle::colour` holds the base colour as `red/green/blue` ints (0–255); divide by 255
to get float channels before multiplying by brightness.

> **Portability (the project is `-std=c++11`):** use `std::min`/`std::max` (from `<algorithm>`),
> not `std::clamp` (C++17). On some compilers `M_PI` isn't predefined — add
> `#define M_PI 3.14159265358979f` or include the right math header if it won't compile.

---

## 3. Global state & conventions (recap)

Defined in `MainApplication.cpp` (already present — reuse, don't reinvent):

```cpp
std::vector<ModelTriangle> triangles;   // the scene
std::vector<Colour>        palette;     // the .mtl colours
glm::vec3 cameraPosition(0.0f, 0.0f, 4.0f);
glm::mat3 cameraOrientation;
glm::vec3 lightPosition(0.0f, 0.8f, 0.0f);   // single point light, near the ceiling
```

**Conventions to keep consistent (04 T5):** positive `z` is *towards* the camera, negative `z`
*away*. Focal length `f = 2.0`, camera at `z = 4.0`. `ModelTriangle::normal` is **precomputed once
at load time** (normals don't change) — that's the field 07 T3 tells you to fill in.

---

## 4. Workbook 07 — Lighting and Shading

Each effect is a *multiplier on the base colour*, composed together. Build them in order and turn
each on with a key so you can see it in isolation (the workbook repeatedly says "aesthetics rule
over science here").

### 4.1 Proximity lighting (07 T2)

**Goal:** closer surface → brighter. **Formula:** `I = 1 / (4π r²)`, clamped to `[0,1]`.

```cpp
float RayTracedRenderer::proximity(const glm::vec3 &p) const {
    float r = glm::length(lightPosition - p);   // GLM 'length' = magnitude; DO NOT normalise
    float I = 1.0f / (4.0f * M_PI * r * r);     // the 1/4πr² law
    return std::min(1.0f, std::max(0.0f, I));   // clamp into [0,1]
}
```
- **Where it goes:** inside `shadeIntersection`, multiply the base RGB by this.
- **Tips:** try `1/3πr²`, `2/7πr²` etc. and pick what *looks* good. Add a key handler to move
  `lightPosition` around the room (arrow keys) — but only *view* it on the ray tracer.
- **Test:** move the light toward the red box and watch it brighten; the far wall should stay dark.

### 4.2 Angle-of-incidence (diffuse) lighting (07 T3)

**Goal:** light a surface by its *orientation* to the light. **Formula:** `I = max(0, N·L)`.

```cpp
// At load time, per triangle (normals are constant, so do this ONCE in loadGeometry):
glm::vec3 e0 = t.vertices[1] - t.vertices[0];
glm::vec3 e1 = t.vertices[2] - t.vertices[0];
t.normal = glm::normalize(glm::cross(e0, e1));   // GLM cross → surface normal; flip if all-dark

float RayTracedRenderer::diffuse(const ModelTriangle &t, const glm::vec3 &p) const {
    glm::vec3 N = t.normal;
    glm::vec3 L = glm::normalize(lightPosition - p);   // *** light FROM the surface point ***
    return std::max(0.0f, glm::dot(N, L));             // GLM dot → [0,1]
}
```
- **The two traps the workbook flags:**
  1. **Winding order** — pick *which* two edges to cross; if every surface comes out dark, flip
     the cross order (`cross(e1,e0)`).
  2. **Direction of `L`** — use *light-from-point* (`lightPosition - p`), **not** the reverse.
     The reverse lights the *back* faces.
- **Test:** with shadows off, the floor and walls facing the light brighten; move the light and
  the bright patch should track it.

### 4.3 Specular lighting (07 T4)

**Goal:** a tight "shiny" highlight. **Formula:** `I = (R·V)^n`, `R = 2(N·L)N − L`, `n ≈ 256`.

```cpp
float RayTracedRenderer::specular(const ModelTriangle &t, const glm::vec3 &p) const {
    glm::vec3 N = t.normal;
    glm::vec3 L = glm::normalize(lightPosition - p);
    glm::vec3 V = glm::normalize(cameraPosition - p);      // view dir = camera - point
    glm::vec3 R = L - 2.0f * glm::dot(N, L) * N;            // R = 2(N·L)N − L  (== glm::refl(-L, N))
    float spread = 256.0f;
    return std::pow(std::max(0.0f, glm::dot(R, V)), spread);
}
```
- **Where it goes:** *add* a specular contribution to the colour, usually white:
  `colour += 1.0f * specular(t, p) * vec3(1.0f,1.0f,1.0f)`.
- **Test:** put the camera roughly *opposite* the light so the reflection is visible (a big
  exponent makes a small, hard highlight). Switch the other lights off to see it clearly.

### 4.4 Ambient lighting (07 T5)

**Goal:** a cheap approximation to global illumination so shadows aren't pure black.
**Formula:** a **minimum threshold** via `max`.

```cpp
// after combining proximity*diffuse (+specular):
float ambient = 0.15f;                          // ~0.1–0.2
float brightness = std::max(ambient, combined); // never below the threshold
// equivalently, blend:  final = ambient + (1.0f - ambient) * combined;
```
- **Test:** compare your shadow regions to the 06 no-ambient image — they should be dark-grey,
  not black, everywhere.

### 4.5 Gouraud shading (07 T6)

**Goal:** smooth the blocky low-poly look by **interpolating brightness across each triangle**
using barycentric weights `w0,w1,w2` (from section 1).

```cpp
// Compute brightness AT EACH VERTEX first (reusing your helpers):
float b0 = proximity(t.vertices[0]) * diffuse(t, t.vertices[0]);
float b1 = proximity(t.vertices[1]) * diffuse(t, t.vertices[1]);
float b2 = proximity(t.vertices[2]) * diffuse(t, t.vertices[2]);

// Then, at each hit pixel, get barycentric weights and blend:
glm::vec3 w = barycentric(hit);   // (w0, w1, w2), sum = 1
float brightness = w.x * b0 + w.y * b1 + w.z * b2;
```
- **The trick the workbook flags — *which weight goes to which vertex* — is the #1 source of the
  visible "seams".** The two sources you'll use give the weights in **different orders**, so don't
  copy-paste one into the other:

  | Source | What it returns | `.x / .y / .z` are the weights of |
  |---|---|---|
  | Intersection solve `sol = (t,u,v)` | `w0=1-u-v, w1=u, w2=v` | **v0 / v1 / v2** (in order) |
  | `Utils::convertToBarycentricCoordinates(v0,v1,v2,r)` | `glm::vec3(u, v, w)` | **v1 / v2 / v0** (note the swap!) |

  So if you *do* use the 2D helper, its `.x` multiplies `b1` (vertex 1), `.y` multiplies `b2`
  (vertex 2), and `.z` multiplies `b0` (vertex 0):
  `float brightness = r.y * b1 + r.z * b2 + r.x * b0;` — **not** `r.x*b0 + r.y*b1 + r.z*b2`.
  In the ray tracer it's simplest to skip the helper and use the intersection's `w0/w1/w2` directly
  (in v0/v1/v2 order), matching the code above.
- **Where the weights come from:** for the *rasteriser* you can use the provided
  `Utils::convertToBarycentricCoordinates(v0,v1,v2,r)` (2D `vec2`, returns `(u,v,w)` as above). For
  the *ray tracer* you already have the weights from the intersection solve: `w0 = 1-u-v, w1 = u,
  w2 = v` (v0/v1/v2 order) — use these to avoid the ordering trap entirely.
- **Test:** load the provided **`sphere.obj`** (07 T6) — a sphere shows the smoothing far better
  than the Cornell Box.

### 4.6 Phong shading (07 T7)

**Goal:** smooth the *surface normal* across the triangle, then light it — fixes the bright "cross"
seams you see at face boundaries with Gouraud.

```cpp
// 1. Interpolate the NORMAL across the face (using the same barycentric weights):
glm::vec3 N = glm::normalize(w.x * t.vertices[0].normal
                           + w.y * t.vertices[1].normal
                           + w.z * t.vertices[2].normal);
// 2. Light using that smooth normal instead of the flat per-triangle normal:
glm::vec3 L = glm::normalize(lightPosition - p);
glm::vec3 V = glm::normalize(cameraPosition - p);
glm::vec3 R = L - 2.0f * glm::dot(N, L) * N;            // R = 2(N·L)N − L
float I = proximity(p) * std::max(0.0f, glm::dot(N, L))
        + 1.0f * std::pow(std::max(0.0f, glm::dot(R, V)), 256.0f);
```
- **Note:** Phong needs a **normal per vertex**. The OBJ's per-triangle normal won't do — derive a
  per-vertex normal by averaging the normals of every triangle that shares that vertex (do this in
  `loadGeometry`), or recompute per vertex at load time.
- **Test:** the specular highlight should slide *continuously* across faces instead of snapping
  at each triangle edge.

---

## 5. Workbook 08 — Coursework Advanced Topics

These are *optional stretch techniques*. The workbook says implementing a selection of them "should
result in a higher grade (depending on how well you implement them)" and that **harder techniques
carry more marks** — check the **Indicative Marking Guide** for the exact weighting. Suggested
order below is roughly by difficulty and by how much each reuses earlier work.

> **Two notes before you start 08.** (1) Unlike 07, the 08 task *text* files don't print the exact
> formulas — reflection and Snell's law live only in the **slide PDFs / audio / animation**. The
> forms below are the standard textbook versions the slides imply; confirm the details (normal
> orientation, `eta = n1/n2`, Fresnel) against the slides. (2) GLM ships `glm::refl(I,N)` and
> `glm::refract(I,N,eta)` — but the material points you at the **manual dot-based** formula
> (`R = D − 2(N·D)N`), which is what you'll be marked on. GLM's spelling is `refl` (not `reflect`).

### 5.1 Texture mapping (08 T2) — the "free lunch"

**Goal:** perspective-correct textures, "for free" because you already have barycentric weights.
**Formula:** interpolate the *texture coordinates* of the three vertices with `w0,w1,w2`.

```cpp
glm::vec3 b = barycentric(hit);                 // (w0, w1, w2)
float tx = b.x * t.texturePoints[0].x + b.y * t.texturePoints[1].x + b.z * t.texturePoints[2].x;
float ty = b.x * t.texturePoints[0].y + b.y * t.texturePoints[1].y + b.z * t.texturePoints[2].y;
// sample the TextureMap (wrap or clamp as you like), then multiply by your lighting brightness:
// uint32_t texel = texture.pixels[ ... (tx,ty) ... ];
// pixel = brightness * unpack(texel);
```
- **Why it's easy here:** the ray tracer's `u,v,w` already give perspective-correct interpolation —
  exactly the result the 04/05 rasteriser could only get with a hard perspective-correction
  formula. *"Barycentric coordinates are great — once you have them, perspective-correct textures
  come for free."*
- **Test:** map a checker/grating texture; the strips should stay *straight* (not fan) under
  perspective.

### 5.2 Soft shadows (08 T3)

**Goal:** blur hard shadow edges by treating the light as an *area* (disc) source — sample many
sub-rays and average.

```cpp
// At each hit point p, cast K shadow rays toward random points on a small disc around the light:
float lit = 0.0f;
const int K = 16;
for (int i = 0; i < K; i++) {
    glm::vec2 d = randomDiscPoint();                       // radius = the light's "size"
    glm::vec3 jitteredLight = lightPosition + glm::vec3(d.x, 0.0f, d.y);
    if (canSeeLight(p, jitteredLight, hit.triangleIndex)) lit++;   // your 06 shadow test
}
float shadow = lit / K;                                     // 0.0 = fully shadowed, 1.0 = fully lit
```
- **Where it goes:** multiply the diffuse/proximity term by `shadow` (and keep ambient). The cost
  is `×K` the number of shadow tests — `K=16` already makes this noticeably slower; use the fast
  renderer to move the camera, then ray-trace to view.
- **Test:** the shadow boundary should be a *gradient* (penumbra), not a crisp edge.

### 5.3 Mirrored surfaces (08 T4)

**Goal:** reflect the camera ray off a "glassy" surface and shade whatever it hits next.
**Formula:** `R = d − 2(N·d)N = glm::refl(d, N)`; recurse.

```cpp
uint32_t RayTracedRenderer::shadeIntersection(const RayTriangleIntersection &hit, int depth) {
    uint32_t base = /* your lit colour from 07 */;
    if (depth > MAX_DEPTH) return base;                 // MUST have a recursion limit
    glm::vec3 N = hit.intersectedTriangle.normal;
    // Manual form the course expects (R = D − 2(N·D)N). rayDirection points camera→surface.
    glm::vec3 R = rayDirection - 2.0f * glm::dot(N, rayDirection) * N;   // == glm::refl(rayDirection, N)
    glm::vec3 origin = hit.intersectionPoint + N * 1e-3f;  // ε offset to avoid self-intersection
    RayTriangleIntersection next = getClosestValidIntersection(origin, R, /*skip?*/);
    uint32_t reflected = (next.found) ? shadeIntersection(next, depth + 1) : background;
    float k = 0.5f;                                     // reflectivity
    return blend(base, reflected, k);                   // pixel = (1-k)*base + k*reflected
}
```
- **Where it goes:** at the top of `shadeIntersection`, *before* returning the base colour.
- **Test:** place a mirrored triangle between the camera and the boxes; you should see the boxes
  reflected. Add a reflectivity blend and a depth cap — without the cap it never terminates.

### 5.4 Environment maps (08 T5)

**Goal:** avoid tracing reflections to the *whole* scene; sample a flat "sky" image instead.

```cpp
// Map the reflected direction R onto an equirectangular image:
float u = std::atan2(R.y, R.x) / (2.0f * M_PI) + 0.5f;
float v = std::asin(std::min(1.0f, std::max(-1.0f, R.z))) / M_PI + 0.5f;
uint32_t envTexel = envMap.pixels[(int)(v * envMap.height) * envMap.width + (int)(u * envMap.width)];
return blend(base, envTexel, reflectivity);
```
- (Cube maps are the alternative: pick the largest-magnitude component of `R` to choose a face.)
- **Where it goes:** replaces the recursive reflect in 5.3 with a texture lookup — much cheaper.
- **Test:** a mirrored surface should show the environment image, not the scene.

### 5.5 Metallic surfaces (08 T6)

**Goal:** mirrors that *tint* the reflection and look rough, not glassy.

```cpp
// 1. Tint the reflection by the metal's colour (gold/brass/copper = its base albedo):
glm::vec3 reflected = tint(reflectedColour, metalColour);   // multiply RGB by the metal colour
// 2. Perturb the reflection to fake micro-roughness (roughness 0.0–1.0):
glm::vec3 jitter = smallRandomVector() * roughness;        // deterministic per-pixel/point!
glm::vec3 Rrough = glm::normalize(R + jitter);
```
- **The failure mode the workbook warns about:** if the jitter changes every frame the surface
  "swirls"; if it's too big it "speckles". Make the perturbation **deterministic** (seed it from
  a hash of the intersection point or pixel, not `rand()` per frame) and keep it **small**.
- **Test:** a gold plane should reflect a warm, slightly blurred version of the scene — steady,
  not flickering.

### 5.6 Refractive materials (08 T7) — glass / water

**Goal:** transparent surfaces that *bend* light. Both **refraction** *and* **reflection** matter;
at grazing angles the surface acts as a mirror (**total internal reflection**).

```cpp
float n1 = 1.0f, n2 = 1.5f;                      // air → glass
float eta = n1 / n2;
glm::vec3 I = glm::normalize(p - lightPosition);  // incident ray (light → surface)
glm::vec3 N = hit.intersectedTriangle.normal;
if (glm::dot(I, N) > 0.0f) N = -N;               // N must oppose the incident ray before refract
glm::vec3 T = glm::refract(I, N, eta);           // manual: Snell's law n1 sinθ1 = n2 sinθ2; returns 0 on TIR
float fresnel = /* Schlick: F0 + (1-F0)*(1 - (N·V))^5, F0 ≈ 0.02–0.04 */;
uint32_t reflected = (depth < MAX_DEPTH) ? traceReflected(...) : 0;
uint32_t refracted = (depth < MAX_DEPTH) ? traceRefracted(...) : 0;
return fresnel * reflected + (1.0f - fresnel) * refracted;
```
- **The gotcha the workbook flags:** at *grazing* (shallow) angles the refracted ray can't escape
  the block — **total internal reflection** — so the surface acts as a mirror there (that's exactly
  why you mix a reflection term with the Fresnel weight). Validate against the online optics tool
  and `resources/reference.jpg`.
- **Test against references:** the workbook gives an online optics tool
  (ricktu288.github.io/ray-optics) and a physical Cornell-box reference image — use both to
  validate. Watch for internal reflection at grazing incidence.
- **This is the hard one** — recursion + Fresnel + TIR. Attempt it last.

### 5.7 Depth of field / bokeh (08 T8)

**Goal:** blur what's outside the focus zone by simulating an **aperture** — Monte-Carlo average
many rays per pixel through random points on the aperture disc.

```cpp
// Per pixel, fire N rays, each through a random point on the aperture disc at the focal plane:
uint32_t accum = 0;
const int N = 64;
for (int i = 0; i < N; i++) {
    glm::vec2 ap = randomDiscPoint() * apertureRadius;      // aperture radius = "f-number"
    glm::vec3 from = cameraPosition + cameraOrientation * glm::vec3(ap.x, ap.y, 0);
    glm::vec3 dir = glm::normalize(pixelpoint - from);      // aim at the pixel, not the focus
    accum += tracePrimaryRay(from, dir);                    // average over all samples
}
return accum / N;                                            // out-of-focus → blurred (bokeh)
```
- **Focal distance** = where the aperture disc sits in `z`; rays through it all hit the *same*
  point on that plane → sharp; everything else spreads → blur.
- **Stretch:** hexagonal aperture (DSLR look), or a swirl/catadioptric filter.
- **Cost:** `×N` full ray traces *per pixel* — the slowest effect here.

### 5.8 Photon maps (08 T9) — global illumination, precomputed

**Goal:** precompute light transport once, reuse it every frame — far cheaper than ambient for
indirect lighting / caustics.

```
Pass 1 (emission, ONCE):  shoot many photons from the light source into the scene;
                          where each photon hits, store it in a spatial structure (a 3D grid).
Pass 2 (collection, per frame):  for each camera ray / pixel, query the grid for nearby photons
                          and sum their contribution → irradiance (soft GI, caustics).
```
- **Where it goes:** a precompute step at scene load + a grid query inside `shadeIntersection`.
- **Reality check:** genuinely advanced. Treat as a top-tier stretch goal; a correct *ambient*
  threshold (07 T5) is the acceptable fallback for most renders.

---

## 6. Suggested schedule

Treat 07 as the **core** (you'll almost certainly be marked on it) and 08 as **stretch** (pick 2–3,
weighted by difficulty/marks).

| Phase | Work | Target |
|---|---|---|
| **A** | 07 T2–T4 proximity + diffuse + specular, each behind its own key toggle | core lighting works, looks plausible |
| **B** | 07 T5 ambient threshold | shadows look natural |
| **C** | 07 T6 Gouraud, then T7 Phong (per-vertex normals) on `sphere.obj` | smooth shading |
| **D** | 08 T2 texture mapping (cheap, high value) | textured model |
| **E** | 08 T3 soft shadows and/or T4 mirrors (with a depth cap) | one "wow" effect |
| **F** | 08 T6 metals / T7 refraction / T8 DoF / T9 photons — pick what the marks guide rewards | stretch |
| **G** | Polish: final camera, final light, save a clean `saved-frames/*.bmp` render | submission |

Keep one working commit per phase. Use `make speedy` only for the final *view*; develop in
`make debug`/`diagnostic`.

---

## 7. Testing & debugging (from `Debugging/README.md`)

- **Single-pixel isolation:** add a mouse-click handler that prints the clicked `(x,y)`; constrain
  the main loop to that one pixel and print *every* intermediate (`t,u,v`, `N`, `L`, `R`, each
  brightness term, shadow-ray hits). Far easier than 77k pixels of output.
- **Visual debugging:** ignore surface colour and instead encode a *number* as colour — e.g. paint
  each pixel by its distance-from-camera or by its diffuse `N·L`. A wrong term shows up as an
  obvious pattern, not a stream of digits.
- **Cut the model down:** render one or two triangles fully (the rest as wireframe) to see what's
  happening.
- **Always cap recursion** (mirrors, refraction) and always keep the fast renderer to *navigate*,
  ray-trace to *view*.

---

## 8. Stretch goals & marking

- The 08 workbook is a *playlist*; the **Indicative Marking Guide** tells you which techniques
  carry more marks (harder ⇒ more marks). Target the high-value ones you can actually finish:
  texture mapping (cheap, visible), soft shadows, and one reflective/refractive effect usually beat
  a shaky photon-map attempt.
- Whatever you choose, the **07 core (proximity → diffuse → specular → ambient → Gouraud/Phong)
  must be solid first** — the advanced effects all reuse it.
- **Submit:** the `RenderProject` source (your `RayTracedRenderer.*` + `MainApplication.cpp`),
  plus one or two saved frames from `saved-frames/` showing each effect you implemented.

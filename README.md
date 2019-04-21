## Final Stage 2.0
Final Stage is a basic path tracer that demonstrates some of the most popular rendering features. The first version of Final Stage was completed in 2011 and focused on programmable shading and GPU acceleration (via CUDA). For version 2 I sought to simplify the code and focus on broad platform support. For more information on the technical design of this version check out its [project page](http://bertolami.com/index.php?engine=portfolio&content=graphics&detail=final-stage-2-0).

### Features
* **Physically based rendering** using an unbiased simulation of light transport that incorporates both surface and subsurface interactions with materials.
* **Optimized scene traversal** using octrees with leaf-neighbor traversal to improve scene collision work by up to 100x over naïve approaches.
* **Fullscreen antialiasing** using multiple rays per pixel to improve overall scene quality. Supports an arbitrary number of multi-rays, with a default of 4.
* **Variance adaptive image denoising** that removes noise produced by the Monte Carlo path sampling process. Improves final output equivalent to a 300% increase in ray samples.
* **Full 128 bit color pipeline** with support for high dynamic range textures and effects.
* **Configurable image plane effects** designed to increase realism and stylistic control, including depth of field, flare, and bloom.
* **Configurable material library** that supports standard properties for describing surface/light interactions, including ambient, diffuse, and emissive terms, solid angle thresholds, and subsurface density. Also includes a higher level layer of intuitive predefined materials including liquid, glass, ceramic, metal, clay, mirror, and fog.
* **Simple scene descriptions** using Wavefront object (.obj) files, Microsoft bitmap (.bmp) files, and HDRsoft’s high dynamic range image (.hdr) files, as well as native primitive shapes including sphere, plane, and cuboid.
* **Multithreading support** using C++11 threads.

### Screenshots
![Screenshot](https://github.com/ramenhut/final-stage-path-tracer-2-0/raw/master/thumbnails/fs2-1-s.jpg?raw=true)
![Screenshot](https://github.com/ramenhut/final-stage-path-tracer-2-0/raw/master/thumbnails/fs2-15-s.jpg?raw=true)
![Screenshot](https://github.com/ramenhut/final-stage-path-tracer-2-0/raw/master/thumbnails/fs2-26-s.jpg?raw=true)

### License

This software is released under the terms of the BSD 2-Clause “Simplified” License.

### More Information
For more information, including pre-built binaries, visit [http://www.bertolami.com](http://bertolami.com/index.php?engine=portfolio&content=graphics&detail=final-stage-2-0).

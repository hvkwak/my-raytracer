# My Ray Tracer
Implementation of ray tracing (based on the course *Computer Graphics (Graphische Datenverarbeitung)* @ Computer Graphics Group, TU Dortmund), BVH, and acceleration with CUDA

## Note
⚠️ The official exercise codebase is **copyrighted** by the **Computer Graphics Group, TU Dortmund** and **is not included** in this repository.

## Updates
- [2025-11-04] Mesh and Triangle attributes can now be saved in a SoA (Structure-of-Arrays) layout for coalesced memory access.
- [2025-10-27] Implemented basic BVH (Bounding Volume Hierarchy) with my median-splitter. Ref: [jbikker](https://github.com/jbikker/bvh_article)
- [2025-10-20] Implemented all TODOs in the course exercises (Phong lighting model, reflections, intersections, flat and Phong shading, textures, acceleration with axis-aligned bounding box (AABB) tests) for ray tracing.

## Outputs
Below is a subset of the results that can be found in the [outputs](outputs/).
<table>
  <tr>
    <th colspan="3" align="left">Ray tracing with Phong lighting</th>
  </tr>
  <tr>
    <td align="center">
      <a href="outputs/o_01_spheres.png"><img src="outputs/o_01_spheres.png" alt="spheres" width="320"></a><br>
      <sub>o_01_spheres.png</sub>
    </td>
    <td align="center">
      <a href="outputs/o_04_molecule.png"><img src="outputs/o_04_molecule.png" alt="molecule" width="320"></a><br>
      <sub>o_04_molecule.png</sub>
    </td>
    <td align="center">
      <a href="outputs/o_03_mirror.png"><img src="outputs/o_03_mirror.png" alt="mirror" width="320"></a><br>
      <sub>o_03_mirror.png</sub>
    </td>
  </tr>
  <tr>
    <th colspan="3" align="left">Ray tracing with triangle meshes</th>
  </tr>
  <tr>
    <td align="center">
      <a href="outputs/o_09_rings.png"><img src="outputs/o_09_rings.png" alt="rings" width="320"></a><br>
      <sub>o_09_rings.png</sub>
    </td>
    <td align="center">
      <a href="outputs/o_08_office.png"><img src="outputs/o_08_office.png" alt="office" width="320"></a><br>
      <sub>o_08_office.png</sub>
    </td>
    <td align="center">
      <a href="outputs/o_07_toon_faces.png"><img src="outputs/o_07_toon_faces.png" alt="molecule" width="320"></a><br>
      <sub>o_07_toon_faces.png</sub>
    </td>
  </tr>
  <tr>
    <th colspan="3" align="left">Ray tracing with textures</th>
  </tr>
  <tr>
    <td align="center">
      <a href="outputs/o_10_pokemon.png"><img src="outputs/o_10_pokemon.png" alt="pokemon" width="320"></a><br>
      <sub>o_10_pokemon.png</sub>
    </td>
    <td align="center">
    </td>
    <td align="center">
    </td>
  </tr>
</table>
<!--
Phong lighting model and reflections with spheres `o_01_spheres.png`.
<div style="display: inline-block; vertical-align: top;">
    <img src="outputs/o_01_spheres.png" alt="o_01_spheres.png" width="250"><br>
    <strong>Phong Lighting Model for spheres</strong><br>
  </div>
--->
<!-------
<h2>Outputs</h2>
Below is a subset of the results that can be found in the [outputs](outputs/).
<table>
  <tr>
    <td align="center">
      <a href="outputs/img1.png"><img src="outputs/thumbs/img1_480.png" alt="Caption 1" width="320"></a><br>
      <sub>Caption 1</sub>
    </td>
    <td align="center">
      <a href="outputs/img2.png"><img src="outputs/thumbs/img2_480.png" alt="Caption 2" width="320"></a><br>
      <sub>Caption 2</sub>
    </td>
    <td align="center">
      <a href="outputs/img3.png"><img src="outputs/thumbs/img3_480.png" alt="Caption 3" width="320"></a><br>
      <sub>Caption 3</sub>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="outputs/img4.png"><img src="outputs/thumbs/img4_480.png" alt="Caption 4" width="320"></a><br>
      <sub>Caption 4</sub>
    </td>
    <td align="center">
      <a href="outputs/img5.png"><img src="outputs/thumbs/img5_480.png" alt="Caption 5" width="320"></a><br>
      <sub>Caption 5</sub>
    </td>
    <td align="center">
      <a href="outputs/img6.png"><img src="outputs/thumbs/img6_480.png" alt="Caption 6" width="320"></a><br>
      <sub>Caption 6</sub>
    </td>
  </tr>
  <tr>
    <td align="center">
      <a href="outputs/img7.png"><img src="outputs/thumbs/img7_480.png" alt="Caption 7" width="320"></a><br>
      <sub>Caption 7</sub>
    </td>
    <td align="center">
      <a href="outputs/img8.png"><img src="outputs/thumbs/img8_480.png" alt="Caption 8" width="320"></a><br>
      <sub>Caption 8</sub>
    </td>
    <td align="center">
      <a href="outputs/img9.png"><img src="outputs/thumbs/img9_480.png" alt="Caption 9" width="320"></a><br>
      <sub>Caption 9</sub>
    </td>
  </tr>
</table>
------------------>

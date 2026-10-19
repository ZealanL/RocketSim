# RocketSim viewer licenses and asset records

The viewer contains source code and bundled assets. They do not all use the
same terms. The package license file points to this document to preserve
these distinctions. This document does not grant new permissions.

## Source code

The Rust source uses the MIT license in [`LICENSE`](LICENSE). Preserve that
license text when you redistribute the source or a viewer build. It keeps the
original RocketSim copyright notice for ZealanL and the notice for VirxEC.
The original MIT notice is recorded in
[commit `a6865698`](https://github.com/ZealanL/RocketSim/blob/a6865698cac324b9d147f8ff9a81f0c819057066/LICENSE).

Imported shader source has additional records below. Dependencies and bundled
fonts have their own license terms. This document does not replace their
notices. Preserve the notices required by the dependencies in your build.

## Bundled models

ZealanL made the OBJ models in `src/models/`. Their notice requires credit
and says not to sell the models. The exact terms are in
[`MODEL_LICENSE_README.md`](src/models/MODEL_LICENSE_README.md). Release
archives also include that file at the archive root.

Credit: **model made by ZealanL**.

These terms apply to the ball, car, and boost-pad OBJ models. Do not describe
these models as MIT-licensed. Confirm that your planned distribution complies
with their separate terms.

## Bundled textures

The viewer includes these PNG files in `src/textures/`:

- `ball.png`
- `boost_glow.png`
- `boost_pad.png`
- `car_blue.png`
- `car_orange.png`
- `white.png`

Git records ZealanL as the contributor who added these files in
[commit `df464a9d`](https://github.com/ZealanL/RocketSim/commit/df464a9d).
The project MIT license was present at that commit. The later port in
[commit `71ae1fd5`](https://github.com/ZealanL/RocketSim/commit/71ae1fd5)
calls the imported textures "ZealanL PNGs".

The repository has no texture-specific license or external source record.
The model notice does not state terms for textures. Texture redistribution
terms remain unconfirmed. Confirm these terms before you redistribute a
viewer build that contains the textures. Do not infer texture permissions
from the model notice.

## Shader source records

The source credits the `main_*` and `arena_*` shaders to ZealanL. Git records
their addition in commit `df464a9d`. Commit `71ae1fd5` records their later
port. The debug shaders were added by VirxEC in commit `21eecae9`.

`src/shaders/line_frag.glsl` cites
[Sanya Pushkar's Stack Overflow answer](https://stackoverflow.com/a/56467661)
in its point-to-segment distance helper. That answer was posted in 2019.
Stack Overflow's [content license rules](https://stackoverflow.com/help/licensing)
assign [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/) to
answers from that period. The answer also credits
[meowgoesthedog's earlier answer](https://stackoverflow.com/a/54442561).

This record preserves the reference. It does not establish whether the shader
adapts the answer's code or only uses the same geometry method. Confirm the
attribution and applicable terms before redistribution. Do not remove the
source reference or assume that the project MIT notice resolves this question.

## External simulation and demo assets

Collision meshes are external inputs. The published crates and viewer
archives do not include them. Check their source and license before any
separate redistribution.

The Zero Two OBJ used by the debug-rendering demo is user-local input. Its
redistribution license is unconfirmed. The published crates and viewer
archives do not include that OBJ or assets derived from it.

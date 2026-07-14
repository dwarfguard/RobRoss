# CAD Assets

Mechanical prototypes for the canvas holder and paint holder. The source files
were created with **SOLIDWORKS 2025 SP5.0**; older versions may not open them.

## Assemblies

| Assembly | Entry point | Notes |
| --- | --- | --- |
| Canvas holder v2 | `prototypes/canvas_Holder/version_2/Canvas_V2.SLDASM` | Latest canvas-holder revision in this repository. |
| Canvas holder v1 | `prototypes/canvas_Holder/version_1/Canvas_V1.SLDASM` | Earlier revision. |
| Paint holder v1 | `prototypes/paint_Holder/version_1/Paint_Holder.SLDASM` | Paint-holder prototype. |

## Opening Files

Clone or download the complete `CAD/` directory and preserve its folder
structure. SOLIDWORKS assemblies reference sibling part files and will report
missing components if only the `.SLDASM` file is downloaded.

Canvas-holder assemblies also use the shared
[`Canvas.SLDPRT`](prototypes/canvas_Holder/Canvas.SLDPRT) model. Keep it in its
current location relative to the version folders.

No neutral STEP or STL exports are currently included.

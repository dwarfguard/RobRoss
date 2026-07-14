# Artwork Generation

Each subdirectory contains one self-contained artwork-generation approach with
its scripts, tests, and documentation.

| Module | Purpose | Status |
| --- | --- | --- |
| [mondrian](mondrian/README.md) | Generate Mondrian-style artwork, robot paths, and SVG previews. | Active Demo v1 pipeline |

New generation approaches should be added as sibling directories instead of
being folded into `mondrian/`.

Scripts are run from the repository root. They read profiles from `configs/`
and write generated files to `output/`; see the
[Mondrian pipeline guide](mondrian/README.md) for commands.

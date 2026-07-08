# STag codebooks

These `HD*.txt` files are vendored, unmodified codebooks for the STag fiducial
marker system. Each file contains one line per marker id; each line is a
48-character bitstring (the 48 code bits of that marker, in the order used by
`codeLocs` in the reference marker generator). The line number (0-indexed) is
the marker id.

The number after `HD` is the Hamming distance guaranteed between any two
codes in that codebook - higher HD means fewer available marker ids but more
robustness against bit misreads. `generate_stag_markers.py` uses these files
to render printable markers.

Source: https://github.com/ManfredStoiber/stag, path `ref/marker_generator/`,
commit `6214edc03095b6c488d3d95d866aa1a84228e574`.

License: MIT
Copyright (c) 2019 Burak Benligiray
Copyright (c) 2023 Manfred Stoiber

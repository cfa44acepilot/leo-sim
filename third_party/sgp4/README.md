# Vendored SGP4 (David Vallado reference implementation)

`SGP4.h` / `SGP4.cpp` are David Vallado's public-domain reference SGP4, version
**2020-07-13**, taken verbatim from the CelesTrak distribution
(https://celestrak.org/software/vallado/cpp.zip, `cpp/SGP4/SGP4/`).

Unmodified. Used by `src/propagator.cpp` only — the SGP4 header is deliberately
kept out of the rest of the codebase (it is a private dependency behind a pImpl).

We initialize each satellite by calling `SGP4Funcs::sgp4init` directly with the
mean elements (unit-converted exactly as Vallado's `twoline2rv` does); we do
**not** reconstruct or parse TLE strings.

## reference/

`reference/SGP4-VER.TLE` and `reference/00005.e` are the verification element set
and the expected TEME ephemeris for satellite 00005 from the same distribution.
`propagator_test.cpp` checks our output against the t = 0 s and t = 21600 s
(= 360 min) rows of `00005.e` (within ~1 km).


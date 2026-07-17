# References

The implementations and validation methodology use the following primary technical references:

- R. Mahony, T. Hamel, and J.-M. Pflimlin, “Nonlinear Complementary Filters on the Special Orthogonal Group,” *IEEE Transactions on Automatic Control*, 53(5), 2008. DOI: [10.1109/TAC.2008.923738](https://doi.org/10.1109/TAC.2008.923738). An author manuscript is available through [HAL](https://hal.science/hal-00488376).
- J. Solà, “Quaternion kinematics for the error-state Kalman filter,” 2017. [arXiv:1711.02508](https://arxiv.org/abs/1711.02508).
- S. O. H. Madgwick, A. J. L. Harrison, and R. Vaidyanathan, “An Efficient Orientation Filter for Inertial and Inertial/Magnetic Sensor Arrays,” 2010. This is a planned comparison baseline, not currently part of the library.

References describe the mathematical method; they do not imply that the authors endorse this implementation.

The ESKF maps Solà's local quaternion error to the `δθ` block and applies the first-order covariance reset Jacobian after injecting each accepted correction. Gravity is currently treated as a known local-NED vector, so the implementation uses a 16-component nominal state and 15-dimensional error state rather than the optional gravity-augmented 19/18 form.

The process-noise parameters are continuous-time noise densities. Their current
discretization and numerical verification status are recorded in
[`math-audit.md`](math-audit.md).

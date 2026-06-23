# Design note: prior-anchored robust kernels for the Gauss-Newton solver

Status: 
- This RFC explores four approaches (A–D) for prior-anchored robustification
- [PR #71](https://github.com/MOLAorg/mp2p_icp/pull/71) implements only Approach B (prior-referenced per-factor kernel)
- The other approaches remain proposals to explore for future work
Scope: `mp2p_icp/src/optimal_tf_gauss_newton.cpp`, `mp2p_icp/include/mp2p_icp/optimal_tf_gauss_newton.h`,
`mp2p_icp/include/mp2p_icp/robust_kernels.h`, `mp2p_icp/src/Solver_GaussNewton.cpp`.

---

## 1. Problem statement

The current Gauss-Newton (GN) solver minimizes a sum of per-pairing squared residuals plus an
optional Gaussian prior term:

```
x* = argmin_x  Σ_i  w_i · ρ( ‖r_i(x)‖² )   +   e_prior(x)ᵀ Σ_prior⁻¹ e_prior(x)
```

where:

- `r_i(x)` is the residual of pairing *i* (point-to-point, point-to-plane, etc.),
- `ρ(·)` is a robust loss (Geman-McClure or Cauchy), applied through the IRLS
  square-root-weight functor `robustSqrtWeightFunc` in
  `optimal_tf_gauss_newton.cpp`,
- `e_prior(x) = Log(prior.mean⁻¹ ∘ x)` is the SE(3) deviation of the current solution from the
  prior mean, and `Σ_prior⁻¹` is the prior information matrix
  (`OptimalTF_GN_Parameters::prior`).

### What the robust kernel does today

In every place a kernel is applied (see `optimal_tf_gauss_newton.cpp` lines 136-141, 227-234,
315-320, 342-347, 379-384, 439-444) the per-pairing weight is computed **from that pairing's own
residual evaluated at the current linearization point**:

```cpp
double weight = w.pt2pt, retSqrNorm = ret.asEigen().squaredNorm();
if (robustSqrtWeightFunc) { weight *= robustSqrtWeightFunc(retSqrNorm); }
```

This is classic IRLS M-estimation. It is excellent at suppressing **individual** correspondences
that disagree with the **current iterate**. It has two structural blind spots:

1. **It is referenced to the current linearization point, not to any external belief.** If a large
   group of wrong correspondences is *self-consistent*, they collectively drag the iterate to a
   wrong basin. Once there, each individual residual is small again, so the per-factor kernel sees
   nothing wrong and happily keeps the bad solution. The prior term resists this only as a soft
   quadratic spring; it cannot *suppress* the conspiring data, it can only be out-voted by it.

2. **It cannot use the prior as the reference of trust.** In many MOLA use cases the prior is a
   genuinely informative external estimate (IMU pre-integration, wheel odometry, a previous
   localization fix). When the data and the prior strongly disagree, the desired behavior is often
   "trust the prior, distrust the data *as a whole*", not "trust whichever has more factors".

### Goal

Add mechanisms whereby the robust treatment is **anchored to the prior Gaussian** and can affect
**all observations at once** (a global data-vs-prior gate), and/or affect each observation by how
far it sits from the **prior-predicted** geometry (a prior-referenced per-factor kernel) — rather
than only down-weighting factors that diverge from the current linearization point.

This is conceptually the dual of the existing kernel: today we robustify *the prior against the
data* (the prior is just one more quadratic term, easily out-voted); we now also want to robustify
*the data against the prior*.

---

## 2. Mathematical background

Define the prior Mahalanobis discrepancy of the current solution:

```
d²(x) = e_prior(x)ᵀ Σ_prior⁻¹ e_prior(x),   e_prior(x) = Log(prior.mean⁻¹ ∘ x) ∈ ℝ⁶
```

Note `d²` automatically ignores DOFs with zero prior information (zero rows/cols of `Σ_prior⁻¹`),
which matches the existing convention documented in `optimal_tf_gauss_newton.h` ("zeros in the
diagonal mean that those prior coordinates should be ignored").

Under the Gaussian-prior null hypothesis (solution truly drawn from the prior), `d²` is
chi-square distributed with `k` DOF, where `k = rank(Σ_prior⁻¹)` is the number of constrained
dimensions. This gives a principled scale for "how surprised should we be that the data wants to
move us this far from the prior".

The existing `cov2cov_alpha` / Birge-ratio `κ` machinery (lines 287-303 and
`OptimalTF_GN_Parameters` docs) is already a *generalized-/tempered-Bayes* global scaling of one
data block. The proposals below generalize that same idea, but keyed to the **prior discrepancy**
and applicable to **all** data blocks.

---

## 3. Proposed approaches

Four approaches, from simplest to most principled. They are not mutually exclusive; A and B can be
combined, and C/D are drop-in refinements of A.

### Approach A — Global prior-consistency gate (recommended first step)

A single scalar `ω ∈ (0, 1]` multiplies the **entire data contribution** (`H_data`, `g_data`, and
its `errNormSqr`) before the prior term is added:

```
ω = γ( d²(x) )
H = ω · H_data + H_prior
g = ω · g_data + g_prior
```

with a monotonically decreasing gate `γ`. Candidate gates:

- **Geman-McClure-style:** `ω = c² / (c² + d²)²` analog, or simpler
- **Cauchy/Student-style:** `ω = c² / (c² + d²)` — smooth, never exactly zero.
- **Chi-square-calibrated:** `c² = χ²_{k, p}` (e.g. the 0.95 quantile for `k` constrained DOF), so
  the gate is unit-free and self-scaling with the number of constrained dimensions.

Interpretation: when the solution is consistent with the prior (`d²` small), `ω ≈ 1` and the
solver behaves exactly as today. When the data conspires to pull the solution far from the prior,
`ω → 0`, the data is globally suppressed and the prior dominates. This is exactly "affect all
observations that diverge from the prior".

**Convergence note (important).** Making `ω` depend on `x` makes the cost non-quadratic. To keep
each GN step a well-posed linear solve, compute `ω` from the **previous** iterate (lagged weight,
exactly as IRLS already does for per-factor weights). This preserves the existing iteration
structure: `ω` is a constant within one inner iteration.

**Hook point.** This is a small, local change in `optimal_tf_gauss_newton.cpp`. The cleanest
implementation accumulates **all** data blocks into a separate `H_data`/`g_data`
(mirroring how cov2cov is already accumulated separately into `H_cc`/`g_cc`), scales by `ω`, then
adds the prior term. `ω` uses `d²` of `result.optimalPose` from the end of the previous iteration.

Pros: trivial, cheap, robust to coordinated outliers, reuses prior we already compute.
Cons: all-or-nothing on the data; cannot keep *some* good data while rejecting a corrupted subset.

### Approach B — Prior-referenced per-factor kernel

Keep per-factor granularity, but evaluate the residual used for the kernel weight at the **prior
mean pose** (or a blend), instead of the current iterate:

```
r_i^prior = error_i( prior.mean )
w_i = baseWeight · sqrtKernel( ‖r_i^prior‖² )       // weight fixed across inner iterations
```

or a blend controlled by `β ∈ [0,1]`:

```
weightRef² = (1-β)·‖r_i(x)‖² + β·‖r_i^prior‖²
```

`β = 0` recovers today's behavior; `β = 1` judges every factor purely by its agreement with the
prior; intermediate values combine both references. This down-weights individual correspondences
that are inconsistent with the prior **even when they look fine at a (possibly already corrupted)
current iterate**, while still keeping good data that merely refines the prior.

Pros: keeps the desirable selectivity of per-factor M-estimation; robust to coordinated outliers
because the reference is external. Cons: needs the residual evaluated at `prior.mean`
(extra evaluation, or cache it once since `prior.mean` is constant); only meaningful for the DOFs
the prior constrains (a translation-only prior cannot judge a pure-rotation outlier).

### Approach C — Tempered-Bayes data exponent (generalize the existing `cov2cov_alpha`)

Replace the binary "gate" with a continuous **temperature** on the data likelihood exponent,
exactly in the spirit of the existing `cov2cov_alpha` and Birge `κ`:

```
α = α(d²) ∈ (0, 1],   H = α·H_data + H_prior,  g = α·g_data + g_prior
```

The difference from A is bookkeeping/semantics: this slots directly into the existing
generalized-Bayes story already documented in `OptimalTF_GN_Parameters`, and could reuse/extend
`cov2cov_auto_balance_with_prior` so that the **whole** data set (not just cov2cov) is balanced
against the prior using a Birge-style ratio:

```
κ = max(1, d²(x) / k),   α = 1/κ
```

i.e. once the data has pulled us more than the prior's own uncertainty justifies, discount the
data by the excess factor. This is parameter-free (uses `k = rank(Σ_prior⁻¹)`), which is attractive
operationally.

Pros: principled, parameter-free variant available, consistent with existing code. Cons: same
all-or-nothing limitation as A.

### Approach D — Dynamic Covariance Scaling / switchable constraint on the data block

Borrow Sünderhauf's switchable constraints / DCS (well-known in robust SLAM) applied to the data
block as a whole. Introduce a latent switch `s ∈ [0,1]` with penalty `Φ·(1-s)²`; the optimal `s`
has the closed form

```
s* = min( 1,  2Φ / (Φ + d²) )
```

and the data block is scaled by `s*`. `Φ` is the only tuning knob (the "soft inlier budget" in
prior-Mahalanobis units). This is a battle-tested, closed-form, smooth gate and is essentially a
specific choice of `γ` in Approach A with good theoretical backing.

Pros: established theory, single intuitive parameter, closed form. Cons: conceptually identical
machinery to A — best viewed as the recommended *default gate function* for A rather than a
separate code path.

---

## 4. Recommended plan

1. Implement **Approach A** with a pluggable gate function, defaulting to the **DCS** form
   (Approach D) and a chi-square-calibrated `Φ`. This is the smallest, safest, highest-value step
   and directly answers the request.
2. Add **Approach B** (`β` blend) as an independent, optionally-combinable feature for users who
   need to keep partial good data.
3. Expose **Approach C**'s parameter-free Birge auto-balance as the `auto`/default tuning for A,
   reusing the existing `cov2cov_auto_balance_with_prior` naming pattern.

All three are inert when no prior is supplied (`gnParams.prior` is `std::nullopt`), guaranteeing
zero behavior change for the no-prior code path.

---

## 5. API / configuration changes

### 5.1 `robust_kernels.h`

Add a prior-gate enum and factory, mirroring the existing `RobustKernel` / `create_robust_kernel`
style so it is YAML-serializable via `MRPT_ENUM_TYPE`:

```cpp
enum class PriorGate : uint8_t
{
    None = 0,      // no global prior gate (current behavior)
    DCS,           // s = min(1, 2Φ/(Φ+d²))
    Cauchy,        // ω = c²/(c²+d²)
    GemanMcClure,  // ω = c²/(c²+d²)²  (analog)
};

using prior_gate_func_t = std::function<double(double /*d2*/)>;

inline prior_gate_func_t create_prior_gate(PriorGate gate, double param);
```

### 5.2 `OptimalTF_GN_Parameters` (`optimal_tf_gauss_newton.h`)

```cpp
/** Global robust gate on the *whole data block*, keyed to the Mahalanobis
 *  discrepancy d² between the current solution and the prior. When the data
 *  pulls the solution far from the prior, the data contribution to H and g is
 *  scaled down by the gate, so the prior dominates. Inert if no prior given.
 *  The gate is evaluated with a lagged solution (previous inner iteration) to
 *  keep each Gauss-Newton step a well-posed linear solve. */
PriorGate priorGate      = PriorGate::None;
double    priorGateParam = 0.0;   // Φ (DCS) or c (Cauchy/GM); <=0 => auto chi2

/** If true, when priorGate==None but a prior exists, scale the whole data
 *  block by 1/κ with κ = max(1, d²/rank(Σ_prior⁻¹)) (parameter-free Birge). */
bool dataAutoBalanceWithPrior = false;

/** Blend for the per-factor kernel reference between current iterate (0) and
 *  prior-predicted residual (1). 0 = current behavior. Inert if no prior. */
double kernelPriorRefBlend = 0.0;   // β
```

### 5.3 `Solver_GaussNewton` (`Solver_GaussNewton.{h,cpp}`)

Add members + `MCP_LOAD_OPT` / `DECLARE_PARAMETER_OPT` lines, and copy into `gnParams` in
`impl_optimal_pose`, mirroring the existing `robustKernel` / `robustKernelParam` plumbing:

```yaml
# example solver YAML
class_name: mp2p_icp::Solver_GaussNewton
params:
  maxIterations: 25
  robustKernel: RobustKernel::Cauchy
  robustKernelParam: 0.5
  priorGate: DCS              # NEW
  priorGateParam: 0.0        # NEW; 0 => chi2 auto-calibration
  kernelPriorRefBlend: 0.0   # NEW
```

---

## 6. Implementation sketch (`optimal_tf_gauss_newton.cpp`)

Key structural change: accumulate every data block into `H_data`/`g_data`/`chi2_data` (the cov2cov
block already does this; extend it to all blocks), then combine:

```cpp
// --- once, before the iteration loop ---
const prior_gate_func_t priorGate =
    gnParams.prior.has_value()
        ? mp2p_icp::create_prior_gate(gnParams.priorGate, gnParams.priorGateParam)
        : prior_gate_func_t{};

const double priorRank = gnParams.prior.has_value()
    ? effective_rank(gnParams.prior->cov_inv)   // # constrained DOF
    : 0.0;

double dataGate = 1.0;  // lagged; full trust on first iteration

// --- inside the loop, after building H_data/g_data and BEFORE adding prior ---
H.noalias() += dataGate * H_data;
g.noalias() += dataGate * g_data;
errNormSqr  += dataGate * chi2_data;

// ... add prior term (unchanged, full weight) ...

// --- at end of iteration, recompute the lagged gate from the new solution ---
if (gnParams.prior.has_value())
{
    const auto   eP = mrpt::poses::Lie::SE<3>::log(
                          result.optimalPose - gnParams.prior->mean);
    const double d2 = eP.asEigen().transpose() *
                      gnParams.prior->cov_inv.asEigen() * eP.asEigen();
    if (priorGate)            { dataGate = priorGate(d2); }
    else if (gnParams.dataAutoBalanceWithPrior)
    {
        const double kappa = std::max(1.0, d2 / std::max(1.0, priorRank));
        dataGate = 1.0 / kappa;
    }
}
```

For Approach B, evaluate `r_i^prior = error_i(prior.mean)` (cache per block, since `prior.mean`
is constant) and blend with `retSqrNorm` before calling `robustSqrtWeightFunc`.

Notes:
- The lagged `dataGate` keeps the per-step problem quadratic; convergence behaves like standard
  IRLS with a graduated weight.
- Guard against `dataGate → 0` leaving `H` rank-deficient: when a prior exists, `H_prior` keeps the
  constrained DOFs invertible; for unconstrained DOFs the data still enters (their `d²` rows are
  zero, but the global scalar gate would shrink them too — consider gating only the constrained
  subspace if this proves problematic, an open question listed in §8).

---

## 7. Unit test plan

New test file `tests/test-mp2p_optimize_prior_gate.cpp`, registered in `tests/CMakeLists.txt`,
following the gtest + `Solver_GaussNewton` style of `test-mp2p_optimize_with_prior.cpp`. Helper:
build pt2pt pairings from a known ground-truth pose (as in the existing test), then optionally
corrupt a subset.

1. **No-prior regression.** No prior, `priorGate=DCS`. Assert bit-for-bit identical result to
   `priorGate=None` (the gate must be inert without a prior).

2. **Consistent data is untouched.** Clean data whose optimum coincides with the prior mean.
   Assert the gated solver returns the same pose (within tolerance) as the ungated solver — `ω≈1`.

3. **Coordinated-outlier rejection (the headline case).** Generate ~60% inliers consistent with
   ground truth + prior, and ~40% *mutually self-consistent* outliers built from a different
   "wrong" pose far from the prior (this is the case where per-factor IRLS fails). Assert:
   - baseline (`robustKernel=Cauchy`, no prior gate) drifts toward the wrong pose;
   - with `priorGate=DCS`, the solution stays near the inlier/prior pose.
   This directly demonstrates the new capability over the existing kernel.

4. **Per-factor prior-referenced kernel (Approach B).** A handful of gross outlier
   correspondences; `kernelPriorRefBlend=1`. Assert recovered pose matches the inlier ground truth
   and that the outliers' effective weights (exposed via a debug hook or inferred from residuals)
   are near zero.

5. **Partial / zero-information prior DOFs.** Prior constrains translation only
   (`cov_inv` nonzero on the 3 translation diagonal entries, as in `test_opt_prior` case 1).
   Assert the gate reacts to translational divergence but does not spuriously suppress data when
   only rotation differs (rotation is unconstrained by the prior).

6. **Gate monotonicity & analytic values (DCS).** Unit-test `create_prior_gate(DCS, Φ)` directly:
   `γ(0)=1`, monotonically decreasing, `γ(d²)=2Φ/(Φ+d²)` for `d²≥Φ`, and `γ→0` as `d²→∞`.
   Same for Cauchy/GM forms.

7. **Tempered-Bayes limits (Approach C).** With `dataAutoBalanceWithPrior=true`: when `d² < k`
   the data is at full weight (`κ=1`); when `d² ≫ k` the data weight `≈ k/d²`. Assert both
   regimes, and that `α=1` (force) reproduces the plain MAP result while a very strong discount
   reproduces the prior-only result.

8. **Convergence / fixed point.** Clean, prior-consistent data: assert the lagged-weight iteration
   converges (delta below `minDelta`) and that re-running from the converged pose is a fixed point
   (gate stable, pose unchanged).

9. **Determinism vs TBB.** Run case 3 with and without `MP2P_HAS_TBB` (or both reduction paths)
   and assert identical results, since the gate is applied after reduction.

Each test asserts on `mrpt::poses::Lie::SE<3>::log(result.optimalPose - expected).norm()` with the
same tolerance idioms already used in the repo.

---

## 8. Open questions / risks

- **Subspace gating.** Should the global gate scale the whole data block, or only its projection
  onto the prior-constrained subspace? Whole-block is simplest and matches the request ("affect
  all observations"), but can over-suppress DOFs the prior says nothing about. A projected gate is
  more correct but more complex; defer to a follow-up once Approach A is validated.
- **Interaction with `cov2cov_alpha` / Birge.** The cov2cov block already has its own scaling.
  Decide whether the new global data gate multiplies on top of it or replaces it for that block.
- **Rank estimation** of `Σ_prior⁻¹` for the chi-square calibration: count eigenvalues above a
  tolerance vs. count nonzero diagonal entries. The latter matches existing diagonal-only usage in
  tests but is wrong for correlated priors.
- **Defaults.** Ship with `priorGate=None` so existing pipelines are unchanged; document the new
  options in `docs/source/` and the pipeline-editor metadata per `agents.md` update rules.
```

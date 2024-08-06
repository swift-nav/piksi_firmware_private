/*
 * Copyright (C) 2016 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_PVT_ENGINE_PVT_RETURN_CODES_H
#define LIBSWIFTNAV_PVT_ENGINE_PVT_RETURN_CODES_H

namespace pvt_engine {

/** \defgroup pvt_return_codes PVT Engine Return Codes
 * Defines status return codes for the PVT Engine.
 * \{ */

enum PRC {
  // Successes
  RC_S_OK = 0,
  RC_S_VALIDATED_AMBIGUITIES,
  RC_S_MAX_SUCCESS,

  // Information
  RC_I_SKIPPED_DUE_TO_OBS_RATE,
  RC_I_UNSUPPORTED_SBAS_MESSAGE,
  RC_I_PL_NO_OBSERVATIONS,
  RC_I_PL_INVALID_N_FAULT_MAX,
  RC_I_PL_TOO_MANY_FAULT_MODES,
  RC_I_PL_OBSERVABILITY_TOO_POOR,
  RC_I_PL_UNMONITORED_RISK_TOO_HIGH,
  RC_I_PL_SEPARATION_TEST_FAILED,
  RC_I_PL_UNCONVERGED,
  RC_I_MAX_INFO,

  // Warnings
  RC_W_NO_OBS,
  RC_W_INSUFFICIENT_OBS,
  RC_W_PVT_SOLVE_FAILED,
  RC_W_NO_SAT_CONSTRAINED,
  RC_W_SIGNAL_NOT_PRESENT,
  RC_W_IONO_PARAMS_NOT_SET,
  RC_W_NO_VALIDATED_AMBIGUITIES,
  RC_W_NOT_ENOUGH_AMBIGUITIES_TO_VALIDATE,
  RC_W_NOT_ENOUGH_SATELLITES,
  RC_W_NOT_ENOUGH_DF_CARRIER_PHASE,
  RC_W_NOT_ENOUGH_OBSERVATIONS,
  RC_W_NOT_ENOUGH_AMBIGUITIES,
  RC_W_AMBIGUITIES_NOT_VALIDATED,
  RC_W_FAILED_DROP_ONE_TEST,
  RC_W_PREVIOUS_AMBS_NOT_VALIDATED,
  RC_W_SIGNAL_ALREADY_PRESENT,
  RC_W_PDOP_THRESHOLD_EXCEEDED,
  RC_W_GDOP_THRESHOLD_EXCEEDED,
  RC_W_NO_OTL_PARAMS,
  RC_W_OTL_NAN,
  RC_W_LAMBDA_FAILURE,
  RC_W_PL_NO_INTEGRITY_BUDGET,
  RC_W_NO_MATCHING_TRUTH_EPOCH,
  RC_W_MAX_WARNING,

  // Errors
  RC_E_NOTOK,
  RC_E_NOT_IMPL,
  RC_E_FILTER_NOT_INITIALIZED,
  RC_E_STATE_DOESNT_EXIST,
  RC_E_INVALID_FREQUENCY,
  RC_E_INVALID_COMPUTATION,
  RC_E_VARIANCE_LESS_THAN_ZERO,
  RC_E_INSUFFICIENT_SPACE,
  RC_E_INVALID_VALUE,
  RC_E_TOO_MANY_OUTLIERS,
  RC_E_OBSERVABILITY_TOO_POOR,
  RC_E_MDE_TOO_HIGH,
  RC_E_NO_SSR_CORRECTIONS,
  RC_E_MAX_ERROR
};

namespace prc {

/** Checks if the return code is a success.
 *
 * \param rc The return code to check
 * \return True is the return code is a success. False if not.
 */
inline bool success(PRC rc) { return rc <= RC_S_MAX_SUCCESS; }

/** Checks if the return code is an info.
 *
 * \param rc The return code to check
 * \return True is the return code is an info. False if not.
 */
inline bool info(PRC rc) {
  return (rc > RC_S_MAX_SUCCESS) && (rc <= RC_I_MAX_INFO);
}

/** Checks if the return code is a warning.
 *
 * \param rc The return code to check
 * \return True is the return code is a warning. False if not.
 */
inline bool warning(PRC rc) {
  return (rc > RC_I_MAX_INFO) && (rc <= RC_W_MAX_WARNING);
}

/** Checks if the return code is an error.
 *
 * \param rc The return code to check
 * \return True is the return code is an error. False if not.
 */
inline bool error(PRC rc) { return rc > RC_W_MAX_WARNING; }

/** Provides a loggable string for a return code.
 *
 * \param rc The return code
 * \return A string containing the meaning of the return code
 */
inline const char *message(PRC rc) {
  switch (rc) {
    case RC_S_OK:
      return "OK";

    case RC_S_VALIDATED_AMBIGUITIES:
      return "Validated ambiguities found.";

    case RC_I_SKIPPED_DUE_TO_OBS_RATE:
      return "Epoch update skipped due to observation rate.";

    case RC_I_UNSUPPORTED_SBAS_MESSAGE:
      return "SBAS message type not implemented.";

    case RC_I_PL_NO_OBSERVATIONS:
      return "No useful observations.";

    case RC_I_PL_INVALID_N_FAULT_MAX:
      return "N_fault_max out of valid bounds.";

    case RC_I_PL_TOO_MANY_FAULT_MODES:
      return "Total number of monitored fault modes exceeded maximum.";

    case RC_I_PL_OBSERVABILITY_TOO_POOR:
      return "Insufficient observability across fault modes";

    case RC_I_PL_UNMONITORED_RISK_TOO_HIGH:
      return "Integrity risk of unmonitored faults exceeded threshold.";

    case RC_I_PL_SEPARATION_TEST_FAILED:
      return "Solution separation test failures occurred.";

    case RC_I_PL_UNCONVERGED:
      return "Protection level evaluation failed to converge.";

    case RC_W_NO_OBS:
      return "No obs given, not updating this epoch.";

    case RC_W_INSUFFICIENT_OBS:
      return "Insufficient obs given, not updating this epoch.";

    case RC_W_PVT_SOLVE_FAILED:
      return "PVT solve failed, not updating this epoch.";

    case RC_W_NO_SAT_CONSTRAINED:
      return "No Sat able to be constrained";

    case RC_W_NOT_ENOUGH_SATELLITES:
      return "Not enough satellites to perform this computation";

    case RC_W_NOT_ENOUGH_DF_CARRIER_PHASE:
      return "Not enough multi-freq carrier phase";

    case RC_W_IONO_PARAMS_NOT_SET:
      return "Ionosphere parameters have not been set";

    case RC_W_NOT_ENOUGH_OBSERVATIONS:
      return "Not enough observations to perform this computation";

    case RC_W_SIGNAL_NOT_PRESENT:
      return "Specified signal not present";

    case RC_W_NOT_ENOUGH_AMBIGUITIES:
      return "No float ambiguities in this filter";

    case RC_W_NO_VALIDATED_AMBIGUITIES:
      return "No Validated ambiguities found.";

    case RC_W_PREVIOUS_AMBS_NOT_VALIDATED:
      return "Previous Ambs failed to validate";

    case RC_W_NOT_ENOUGH_AMBIGUITIES_TO_VALIDATE:
      return "Not enough ambs to validate";

    case RC_W_AMBIGUITIES_NOT_VALIDATED:
      return "New Ambiguities not validated";

    case RC_W_FAILED_DROP_ONE_TEST:
      return "Failed drop one test";

    case RC_W_SIGNAL_ALREADY_PRESENT:
      return "Specified signal already present";

    case RC_W_PDOP_THRESHOLD_EXCEEDED:
      return "PDOP threshold exceeded, update failed";

    case RC_W_GDOP_THRESHOLD_EXCEEDED:
      return "GDOP threshold exceeded";

    case RC_W_NO_OTL_PARAMS:
      return "No OTL parameters found.";

    case RC_W_OTL_NAN:
      return "NaN passed to OTL apriori model.";

    case RC_W_LAMBDA_FAILURE:
      return "Integer ambiguity resolution attempt failed";

    case RC_W_PL_NO_INTEGRITY_BUDGET:
      return "No leftover integrity budget allocated for ARAIM.";

    case RC_W_NO_MATCHING_TRUTH_EPOCH:
      return "No truth data found when requested.";

    case RC_E_NOTOK:
      return "Not OK";

    case RC_E_NOT_IMPL:
      return "Not implemented";

    case RC_E_FILTER_NOT_INITIALIZED:
      return "Filter not initialized";

    case RC_E_STATE_DOESNT_EXIST:
      return "State has not been initialized or invalid state index";

    case RC_E_INVALID_FREQUENCY:
      return "Invalid Frequency enum";

    case RC_E_INVALID_COMPUTATION:
      return "Computation or numerical error";

    case RC_E_VARIANCE_LESS_THAN_ZERO:
      return "Variance given less than zero";

    case RC_E_INSUFFICIENT_SPACE:
      return "Insufficient space";

    case RC_E_INVALID_VALUE:
      return "Invalid value found (either NaN or inf).";

    case RC_E_TOO_MANY_OUTLIERS:
      return "Too many outliers, epoch rejected.";

    case RC_E_OBSERVABILITY_TOO_POOR:
      return "Observability too poor, epoch rejected.";

    case RC_E_MDE_TOO_HIGH:
      return "Minimum detectable position error too large, epoch rejected.";

    case RC_E_NO_SSR_CORRECTIONS:
      return "No valid SSR corrections for epoch.";

    case RC_S_MAX_SUCCESS:
    case RC_I_MAX_INFO:
    case RC_W_MAX_WARNING:
    case RC_E_MAX_ERROR:
    default:
      return "Invalid return code";
  }
}

}  // namespace prc

// \}
}  // namespace pvt_engine

#endif  // LIBSWIFTNAV_PVT_ENGINE_PVT_RETURN_CODES_H

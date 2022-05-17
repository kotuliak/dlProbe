/**
 * Copyright 2016-2021 ETH Zurich
 *
 * This file is part of dlProbe, a modified version of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include "srsran/srsran.h"

#include "srsran/common/standard_streams.h"
#include "dlProbe/hdr/phy/lte/cc_worker.h"

#define Error(fmt, ...)                                                                                                \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  logger.error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...)                                                                                              \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  logger.warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)                                                                                                 \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  logger.info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)                                                                                                \
  if (SRSRAN_DEBUG_ENABLED)                                                                                            \
  logger.debug(fmt, ##__VA_ARGS__)

#define CURRENT_TTI (sf_cfg_dl.tti)
#define CURRENT_SFIDX (sf_cfg_dl.tti % 10)
#define CURRENT_TTI_TX (sf_cfg_ul.tti)

namespace srsue {
namespace lte {

/************
 *
 * Common Functions
 *
 */

cc_worker::cc_worker(uint32_t cc_idx_, uint32_t max_prb, srsue::phy_common* phy_, srslog::basic_logger& logger) :
  logger(logger)
{
  cc_idx = cc_idx_;
  phy    = phy_;
  nof_sim_ues = phy->args->nof_sim_ues;

  signal_buffer_max_samples = 3 * SRSRAN_SF_LEN_PRB(max_prb);

  for (uint32_t i = 0; i < phy->args->nof_rx_ant; i++) {
    signal_buffer_rx[i] = srsran_vec_cf_malloc(signal_buffer_max_samples);
    if (!signal_buffer_rx[i]) {
      Error("Allocating memory");
      return;
    }
    signal_buffer_tx[i] = srsran_vec_cf_malloc(signal_buffer_max_samples);
    if (!signal_buffer_tx[i]) {
      Error("Allocating memory");
      return;
    }
  }

  if (srsran_ue_dl_init(&ue_dl, signal_buffer_rx, max_prb, phy->args->nof_rx_ant)) {
    Error("Initiating UE DL");
    return;
  }

  if (srsran_ue_ul_init(&ue_ul, signal_buffer_tx[0], max_prb)) {
    Error("Initiating UE UL");
    return;
  }

  phy->set_ue_dl_cfg(&ue_dl_cfg);
  for (int i = 0; i < nof_sim_ues; i++) {
    phy->set_ue_ul_cfg(&ue_ul_cfg[i]);
  }
  
  phy->set_pdsch_cfg(&ue_dl_cfg.cfg.pdsch);
  phy->set_pdsch_cfg(&pmch_cfg.pdsch_cfg); // set same config in PMCH decoder

  // Define MBSFN subframes channel estimation and save default one
  chest_mbsfn_cfg.filter_type    = SRSRAN_CHEST_FILTER_TRIANGLE;
  chest_mbsfn_cfg.filter_coef[0] = 0.1;
  chest_mbsfn_cfg.estimator_alg  = SRSRAN_ESTIMATOR_ALG_INTERPOLATE;
  chest_mbsfn_cfg.noise_alg      = SRSRAN_NOISE_ALG_PSS;

  chest_default_cfg = ue_dl_cfg.chest_cfg;

  // Set default PHY params
  reset();

  if (phy->args->pdsch_8bit_decoder) {
    ue_dl.pdsch.llr_is_8bit        = true;
    ue_dl.pdsch.dl_sch.llr_is_8bit = true;
  }
}

cc_worker::~cc_worker()
{
  for (uint32_t i = 0; i < phy->args->nof_rx_ant; i++) {
    if (signal_buffer_tx[i]) {
      free(signal_buffer_tx[i]);
    }
    if (signal_buffer_rx[i]) {
      free(signal_buffer_rx[i]);
    }
  }
  srsran_ue_dl_free(&ue_dl);
  srsran_ue_ul_free(&ue_ul);
}

void cc_worker::reset()
{
  // constructor sets defaults
  srsran::phy_cfg_t empty_cfg;
  for (int i = 0; i < nof_sim_ues; i++) {
    set_config_nolock(empty_cfg, i);
  }
  
}

void cc_worker::reset_cell_nolock()
{
  cell_initiated = false;
}

bool cc_worker::set_cell_nolock(srsran_cell_t cell_)
{
  if (cell.id != cell_.id || !cell_initiated) {
    cell = cell_;

    if (srsran_ue_dl_set_cell(&ue_dl, cell)) {
      Error("Setting ue_dl cell");
      return false;
    }

    if (srsran_ue_dl_set_mbsfn_area_id(&ue_dl, 1)) {
      Error("Setting mbsfn id");
    }

    if (srsran_ue_ul_set_cell(&ue_ul, cell)) {
      Error("Initiating UE UL");
      return false;
    }

    if (cell.frame_type == SRSRAN_TDD && ue_dl_cfg.chest_cfg.estimator_alg != SRSRAN_ESTIMATOR_ALG_INTERPOLATE) {
      chest_default_cfg.estimator_alg = SRSRAN_ESTIMATOR_ALG_INTERPOLATE;
      srsran::console("Enabling subframe interpolation for TDD cells (recommended setting)\n");
    }

    cell_initiated = true;
  }
  return true;
}

uint32_t cc_worker::get_buffer_len()
{
  return signal_buffer_max_samples;
}

cf_t* cc_worker::get_rx_buffer(uint32_t antenna_idx)
{
  return signal_buffer_rx[antenna_idx];
}

cf_t* cc_worker::get_tx_buffer(uint32_t antenna_idx)
{
  return signal_buffer_tx[antenna_idx];
}

void cc_worker::set_tti(uint32_t tti)
{
  sf_cfg_dl.tti       = tti;
  sf_cfg_ul.tti       = TTI_TX(tti);
  sf_cfg_ul.shortened = false;
}

void cc_worker::set_cfo_nolock(float cfo)
{
  for (int stack_idx = 0; stack_idx < nof_sim_ues; stack_idx++)
    ue_ul_cfg[stack_idx].cfo_value = cfo;
}

float cc_worker::get_ref_cfo() const
{
  return ue_dl.chest_res.cfo;
}

void cc_worker::set_tdd_config_nolock(srsran_tdd_config_t config)
{
  sf_cfg_dl.tdd_config = config;
  sf_cfg_ul.tdd_config = config;
}

/************
 *
 * Downlink Functions
 *
 */

bool cc_worker::work_dl_regular()
{
  bool dl_ack[SRSRAN_MAX_CODEWORDS] = {};

  mac_interface_phy_lte::tb_action_dl_t dl_action = {};

  bool found_dl_grant = false;

  if (!cell_initiated) {
    logger.warning("Trying to access cc_worker=%d while cell not initialized (DL)", cc_idx);
    return false;
  }

  sf_cfg_dl.sf_type = SRSRAN_SF_NORM;

  // Set default channel estimation
  ue_dl_cfg.chest_cfg = chest_default_cfg;

  /* For TDD, when searching for SIB1, the ul/dl configuration is unknown and need to do blind search over
   * the possible mi values
   */
  uint32_t mi_set_len;
  if (cell.frame_type == SRSRAN_TDD && !sf_cfg_dl.tdd_config.configured) {
    mi_set_len = 3;
  } else {
    mi_set_len = 1;
  }

  // Blind search PHICH mi value
  for (uint32_t i = 0; i < mi_set_len && !found_dl_grant; i++) {
    if (mi_set_len == 1) {
      srsran_ue_dl_set_mi_auto(&ue_dl);
    } else {
      srsran_ue_dl_set_mi_manual(&ue_dl, i);
    }

    /* Do FFT and extract PDCCH LLR, or quit if no actions are required in this subframe */
    if (srsran_ue_dl_decode_fft_estimate(&ue_dl, &sf_cfg_dl, &ue_dl_cfg) < 0) {
      Error("Getting PDCCH FFT estimate");
      return false;
    }

    // Look for DL and UL dci(s) if the serving cell is active and it is NOT a secondary serving cell without
    // cross-carrier scheduling is enabled
    if (phy->cell_state.is_active(cc_idx, sf_cfg_dl.tti) and (cc_idx != 0 or not ue_dl_cfg.cfg.dci.cif_present)) {
      set_rnti_mapping();
      found_dl_grant = decode_pdcch_dl() > 0;
    }
  }

  int ra_rnti_counter[11] = {0};

  for (size_t i = 0; i < phy->stacks->size(); i++) {

    srsran_dci_dl_t dci_dl       = {};
    uint32_t        grant_cc_idx = 0;
    bool            has_dl_grant = phy->get_dl_pending_grant(CURRENT_TTI, cc_idx, &grant_cc_idx, &dci_dl, i);

    // If found a dci for this carrier, generate a grant, pass it to MAC and decode the associated PDSCH
    if (has_dl_grant) {
      // Read last TB from last retx for this pid
      for (uint32_t j = 0; j < SRSRAN_MAX_CODEWORDS; j++) {
        ue_dl_cfg.cfg.pdsch.grant.last_tbs[j] = phy->last_dl_tbs[dci_dl.pid][cc_idx][j][i];
      }
      // Generate PHY grant
      if (srsran_ue_dl_dci_to_pdsch_grant(&ue_dl, &sf_cfg_dl, &ue_dl_cfg, &dci_dl, &ue_dl_cfg.cfg.pdsch.grant)) {
        Info("Converting DCI message to DL dci");
        return false;
      }

      // Save TB for next retx
      for (uint32_t j = 0; j < SRSRAN_MAX_CODEWORDS; j++) {
        phy->last_dl_tbs[dci_dl.pid][cc_idx][j][i] = ue_dl_cfg.cfg.pdsch.grant.last_tbs[j];
      }

      // Set RNTI
      ue_dl_cfg.cfg.pdsch.rnti = dci_dl.rnti;

      // Generate MAC grant
      mac_interface_phy_lte::mac_grant_dl_t mac_grant = {};
      dl_phy_to_mac_grant(&ue_dl_cfg.cfg.pdsch.grant, &dci_dl, &mac_grant);

      // Save ACK resource configuration
      srsran_pdsch_ack_resource_t ack_resource = {dci_dl.dai, dci_dl.location.ncce, grant_cc_idx, dci_dl.tpc_pucch};

      // Send grant to MAC and get action for this TB, then call tb_decoded to unlock MAC
      phy->stacks->at(i)->new_grant_dl(cc_idx, mac_grant, &dl_action);

      // Decode PDSCH
      decode_pdsch(ack_resource, &dl_action, dl_ack);

      // Informs Stack about the decoding status, send NACK if cell is in process of re-selection
      if (phy->cell_is_selecting) {
        for (uint32_t i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
          dl_ack[i] = false;
        }
      }

      if (SRSRAN_RNTI_ISRAR(mac_grant.rnti)) {
        if (ra_rnti_counter[mac_grant.rnti] == 0) printf("Available stacks: %d, of which %d stacks listen for RAR; %d stacks following a user\n", unassigned_stacks, rar_stacks, crnti_stacks);
        ra_rnti_counter[mac_grant.rnti]++;
        phy->stacks->at(i)->tb_decoded(cc_idx, mac_grant, dl_ack, ra_rnti_counter[mac_grant.rnti]);
      } else {
        phy->stacks->at(i)->tb_decoded(cc_idx, mac_grant, dl_ack, 0);
      }
    }
  }

  return true;
}

void cc_worker::dl_phy_to_mac_grant(srsran_pdsch_grant_t*                         phy_grant,
                                    srsran_dci_dl_t*                              dl_dci,
                                    srsue::mac_interface_phy_lte::mac_grant_dl_t* mac_grant)
{
  /* Fill MAC dci structure */
  mac_grant->pid  = dl_dci->pid;
  mac_grant->rnti = dl_dci->rnti;
  mac_grant->tti  = CURRENT_TTI;

  for (int i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
    mac_grant->tb[i].ndi         = dl_dci->tb[i].ndi;
    mac_grant->tb[i].ndi_present = (dl_dci->tb[i].mcs_idx <= 28);
    mac_grant->tb[i].tbs         = phy_grant->tb[i].enabled ? (phy_grant->tb[i].tbs / (uint32_t)8) : 0;
    mac_grant->tb[i].rv          = phy_grant->tb[i].rv;
  }

  // If SIB dci, use PID to signal TTI to obtain RV from higher layers
  if (mac_grant->rnti == SRSRAN_SIRNTI) {
    mac_grant->pid = CURRENT_TTI;
  }
}

void cc_worker::set_rnti_mapping()
{
  rnti_mapping.clear();
  dl_rntis.clear();
  unassigned_stacks = 0;
  rar_stacks = 0;
  crnti_stacks = 0;

  for (size_t i = 0; i < phy->stacks->size(); i++) {
    uint16_t dl_rnti = phy->stacks->at(i)->get_dl_sched_rnti(CURRENT_TTI);
    if (dl_rnti == SRSRAN_INVALID_RNTI) continue;

    if (SRSRAN_RNTI_ISSI(dl_rnti)) {
      unassigned_stacks++;
    } else if (SRSRAN_RNTI_ISRAR(dl_rnti)) {
      unassigned_stacks++;
      rar_stacks++;
    } else if (SRSRAN_RNTI_ISUSER(dl_rnti)) {
      crnti_stacks++;
    }

    if (rnti_mapping.find(dl_rnti) != rnti_mapping.end()) {
      rnti_mapping[dl_rnti].push_back(i);
    } else {
      rnti_mapping[dl_rnti] = std::vector<int>();
      rnti_mapping[dl_rnti].push_back(i);
      dl_rntis.push_back(dl_rnti);
    }
  }

  Debug("There is currently %d unassigned stacks and %d out of them are in rar; %d crnti stacks", unassigned_stacks, rar_stacks, crnti_stacks);
}

int cc_worker::decode_pdcch_dl()
{
  int* nof_grants;

  int nof_grants_arr[dl_rntis.size()] = {};
  nof_grants = nof_grants_arr;

  int found_dl_grant = 0;
  int dl_dci_counter = 0;

  if (!dl_rntis.empty()) {
    srsran_dci_dl_t dci_arr[MULTIUE_MAX_UES][SRSRAN_MAX_DCI_MSG];

    Debug("PDCCH looking for rntis=");
    for (uint16_t rnti : dl_rntis) {
      Debug("0x%x ", rnti);
    }

    uint16_t dl_rntis_size = dl_rntis.size();
    uint16_t dl_rntis_arr[dl_rntis_size];
    std::copy(dl_rntis.begin(), dl_rntis.end(), dl_rntis_arr);

    /* Blind search first without cross scheduling then with it if enabled */
    // todo cross scheduling - right now I have removed it
    ue_dl_cfg.cfg.dci.cif_enabled = false;
    ue_dl_cfg.cfg.dci_common_ss   = (cc_idx == 0);
    nof_grants                    = srsran_ue_dl_find_dl_dcis(&ue_dl, &sf_cfg_dl, &ue_dl_cfg, dl_rntis_arr, dl_rntis_size, dci_arr);
    if (nof_grants[0] < 0) {
      Error("Looking for DL grants");
      return -1;
    }

    for (size_t i = 0; i < dl_rntis.size(); i++)
    {
      if (nof_grants[i]) found_dl_grant = 1;

      // If RAR dci, save TTI
      if (nof_grants[i] > 0 && SRSRAN_RNTI_ISRAR(dl_rntis[i])) {
        Debug("setting rar grant %d", CURRENT_TTI);
        phy->set_rar_grant_tti(CURRENT_TTI);
      }

      for (int k = 0; k < nof_grants[i]; k++) {
        // Save dci to CC index
        if (SRSRAN_RNTI_ISSIRAPA(dci_arr[i][k].rnti)) {
          for (int stack_idx : rnti_mapping[dci_arr[i][k].rnti]) {
            phy->set_dl_pending_grant(CURRENT_TTI, dci_arr[i][k].cif_present ? dci_arr[i][k].cif : cc_idx, cc_idx, &dci_arr[i][k], dl_dci_counter, stack_idx);
            dl_dci_counter++;
          }
        } else {
          phy->set_dl_pending_grant(CURRENT_TTI, dci_arr[i][k].cif_present ? dci_arr[i][k].cif : cc_idx, cc_idx, &dci_arr[i][k], dl_dci_counter, rnti_mapping[dci_arr[i][k].rnti][0]);
          dl_dci_counter++;
        }

        // Logging
        if (logger.info.enabled()) {
          char str[512];
          srsran_dci_dl_info(&dci_arr[i][k], str, 512);
          logger.info("PDCCH: cc=%d, %s, snr=%.1f dB, rnti=0x%x", cc_idx, str, ue_dl.chest_res.snr_db, dci_arr[i][k].rnti);
        }
      }
    }
  }

  if (nof_grants != nof_grants_arr) free(nof_grants);

  return found_dl_grant;
}

int cc_worker::decode_pdsch(srsran_pdsch_ack_resource_t            ack_resource,
                            mac_interface_phy_lte::tb_action_dl_t* action,
                            bool                                   mac_acks[SRSRAN_MAX_CODEWORDS])
{
  srsran_pdsch_res_t pdsch_dec[SRSRAN_MAX_CODEWORDS] = {};

  // See if at least 1 codeword needs to be decoded. If not need to be decode, resend ACK
  bool decode_enable                   = false;
  bool tb_enable[SRSRAN_MAX_CODEWORDS] = {};
  for (uint32_t tb = 0; tb < SRSRAN_MAX_CODEWORDS; tb++) {
    tb_enable[tb] = ue_dl_cfg.cfg.pdsch.grant.tb[tb].enabled;
    if (action->tb[tb].enabled) {
      decode_enable = true;

      // Prepare I/O based on action
      pdsch_dec[tb].payload                  = action->tb[tb].payload;
      ue_dl_cfg.cfg.pdsch.softbuffers.rx[tb] = action->tb[tb].softbuffer.rx;

      // Use RV from higher layers
      ue_dl_cfg.cfg.pdsch.grant.tb[tb].rv = action->tb[tb].rv;

    } else {
      // If this TB is duplicate, indicate PDSCH to skip it
      ue_dl_cfg.cfg.pdsch.grant.tb[tb].enabled = false;
    }
  }

  // Run PDSCH decoder
  if (decode_enable) {
    if (srsran_ue_dl_decode_pdsch(&ue_dl, &sf_cfg_dl, &ue_dl_cfg.cfg.pdsch, pdsch_dec)) {
      Error("ERROR: Decoding PDSCH");
    }
  }

  // Generate ACKs for MAC and PUCCH
  uint32_t nof_tb                             = 0;
  uint8_t  pending_acks[SRSRAN_MAX_CODEWORDS] = {};
  for (uint32_t tb = 0; tb < SRSRAN_MAX_CODEWORDS; tb++) {
    // For MAC, set to true if it's a duplicate
    mac_acks[tb] = action->tb[tb].enabled ? pdsch_dec[tb].crc : true;

    // For PUCCH feedback, need to send even if duplicate, but only those CW that were enabled before disabling in th
    // grant
    pending_acks[tb] = tb_enable[tb] ? mac_acks[tb] : 2;

    if (tb_enable[tb]) {
      nof_tb++;
    }
  }

  if (action->generate_ack && nof_tb > 0) {
    phy->set_dl_pending_ack(&sf_cfg_dl, cc_idx, pending_acks, ack_resource);
  }

  if (decode_enable) {
    // Metrics
    dl_metrics_t dl_metrics = {};
    if (ue_dl_cfg.cfg.pdsch.grant.nof_tb == 1) {
      dl_metrics.mcs = ue_dl_cfg.cfg.pdsch.grant.tb[0].mcs_idx;
    } else {
      dl_metrics.mcs = (ue_dl_cfg.cfg.pdsch.grant.tb[0].mcs_idx + ue_dl_cfg.cfg.pdsch.grant.tb[1].mcs_idx) / 2;
    }
    dl_metrics.fec_iters = pdsch_dec->avg_iterations_block / 2;
    phy->set_dl_metrics(cc_idx, dl_metrics);

    // Logging
    if (logger.info.enabled()) {
      char str[512];
      srsran_pdsch_rx_info(&ue_dl_cfg.cfg.pdsch, pdsch_dec, str, 512);
      logger.info("PDSCH: cc=%d, %s, snr=%.1f dB", cc_idx, str, ue_dl.chest_res.snr_db);
    }
  }

  return SRSRAN_SUCCESS;
}

int cc_worker::decode_pmch(mac_interface_phy_lte::tb_action_dl_t* action, srsran_mbsfn_cfg_t* mbsfn_cfg)
{
  srsran_pdsch_res_t pmch_dec = {};

  pmch_cfg.area_id                     = mbsfn_cfg->mbsfn_area_id;
  pmch_cfg.pdsch_cfg.softbuffers.rx[0] = action->tb[0].softbuffer.rx;
  pmch_dec.payload                     = action->tb[0].payload;

  if (action->tb[0].enabled) {
    srsran_softbuffer_rx_reset_tbs(pmch_cfg.pdsch_cfg.softbuffers.rx[0], pmch_cfg.pdsch_cfg.grant.tb[0].tbs);

    if (srsran_ue_dl_decode_pmch(&ue_dl, &sf_cfg_dl, &pmch_cfg, &pmch_dec)) {
      Error("Decoding PMCH");
      return -1;
    }

    // Store metrics
    // Metrics
    dl_metrics_t dl_metrics = {};
    dl_metrics.mcs          = ue_dl_cfg.cfg.pdsch.grant.tb[0].mcs_idx;
    dl_metrics.fec_iters    = pmch_dec.avg_iterations_block / 2;
    phy->set_dl_metrics(cc_idx, dl_metrics);

    Info("PMCH: l_crb=%2d, tbs=%d, mcs=%d, crc=%s, snr=%.1f dB, n_iter=%.1f",
         pmch_cfg.pdsch_cfg.grant.nof_prb,
         pmch_cfg.pdsch_cfg.grant.tb[0].tbs / 8,
         pmch_cfg.pdsch_cfg.grant.tb[0].mcs_idx,
         pmch_dec.crc ? "OK" : "KO",
         ue_dl.chest_res.snr_db,
         pmch_dec.avg_iterations_block);

    if (pmch_dec.crc) {
      return 1;
    }

  } else {
    Warning("Received dci for TBS=0");
  }
  return 0;
}

void cc_worker::update_measurements(std::vector<phy_meas_t>& serving_cells, cf_t* rssi_power_buffer)
{
  // Do not update any measurement if the CC is not configured to prevent false or inaccurate data
  if (not phy->cell_state.is_configured(cc_idx)) {
    return;
  }

  phy->update_measurements(
      cc_idx, ue_dl.chest_res, sf_cfg_dl, ue_dl_cfg.cfg.pdsch.rs_power, serving_cells, rssi_power_buffer);
}

void cc_worker::ul_phy_to_mac_grant(srsran_pusch_grant_t*                         phy_grant,
                                    srsran_dci_ul_t*                              dci_ul,
                                    uint32_t                                      pid,
                                    bool                                          ul_grant_available,
                                    srsue::mac_interface_phy_lte::mac_grant_ul_t* mac_grant,
                                    int                                           stack_idx)
{
  if (mac_grant->phich_available && !dci_ul->rnti) {
    mac_grant->rnti = phy->stacks->at(stack_idx)->get_ul_sched_rnti(CURRENT_TTI);
  } else {
    mac_grant->rnti = dci_ul->rnti;
  }
  mac_grant->tb.ndi         = dci_ul->tb.ndi;
  mac_grant->tb.ndi_present = ul_grant_available;
  mac_grant->tb.tbs         = phy_grant->tb.tbs / (uint32_t)8;
  mac_grant->tb.rv          = phy_grant->tb.rv;
  mac_grant->pid            = pid;
  mac_grant->is_rar         = dci_ul->format == SRSRAN_DCI_FORMAT_RAR;
  mac_grant->tti_tx         = CURRENT_TTI_TX;
}

/************
 *
 * Configuration Functions
 *
 */

/* Translates RRC structs into PHY structs
 */
void cc_worker::set_config_nolock(const srsran::phy_cfg_t& phy_cfg, int stack_idx)
{
  // Save configuration
  ue_dl_cfg.cfg    = phy_cfg.dl_cfg;
  ue_ul_cfg[stack_idx].ul_cfg = phy_cfg.ul_cfg;

  phy->set_pdsch_cfg(&ue_dl_cfg.cfg.pdsch);
}

void cc_worker::upd_config_dci_nolock(const srsran_dci_cfg_t& dci_cfg)
{
  ue_dl_cfg.cfg.dci = dci_cfg;
}

int cc_worker::read_ce_abs(float* ce_abs, uint32_t tx_antenna, uint32_t rx_antenna)
{
  uint32_t sz = (uint32_t)srsran_symbol_sz(cell.nof_prb);
  srsran_vec_f_zero(ce_abs, sz);
  int g = (sz - 12 * cell.nof_prb) / 2;
  srsran_vec_abs_dB_cf(ue_dl.chest_res.ce[tx_antenna][rx_antenna], -80, &ce_abs[g], SRSRAN_NRE * cell.nof_prb);
  return sz;
}

int cc_worker::read_pdsch_d(cf_t* pdsch_d)
{
  memcpy(pdsch_d, ue_dl.pdsch.d[0], ue_dl_cfg.cfg.pdsch.grant.nof_re * sizeof(cf_t));
  return ue_dl_cfg.cfg.pdsch.grant.nof_re;
}

} // namespace lte
} // namespace srsue

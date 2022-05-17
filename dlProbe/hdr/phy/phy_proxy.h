#ifndef SRSUE_PHY_PROXY_H
#define SRSUE_PHY_PROXY_H

#include "phy_common.h"
#include "phy.h"
#include "phy_metrics.h"
#include "srsran/common/block_queue.h"
#include "srsran/common/threads.h"
#include "srsran/common/trace.h"
#include "srsran/interfaces/phy_interface_types.h"
#include "srsran/interfaces/radio_interfaces.h"
#include "srsran/radio/radio.h"
#include "srsran/srslog/srslog.h"
#include "srsran/srsran.h"
#include "dlProbe/hdr/phy/lte/worker_pool.h"
#include "dlProbe/hdr/phy/nr/worker_pool.h"
#include "dlProbe/hdr/phy/ue_lte_phy_base.h"
#include "dlProbe/hdr/phy/ue_nr_phy_base.h"
#include "sync.h"

namespace srsue {

class phy_proxy final : public phy_interface_stack_lte
{
public:

  void init(phy* phy_h_, int index_) {phy_h = phy_h_; index = index_; };

  // MAC -> PHY
  /* Time advance commands */
  void set_timeadv_rar(uint32_t tti, uint32_t ta_cmd) final {  if (last_tti != tti) {phy_h->set_timeadv_rar(tti, ta_cmd);} last_tti = tti;};
  void set_timeadv(uint32_t tti, uint32_t ta_cmd)     final {  if (last_tti != tti) {phy_h->set_timeadv(tti, ta_cmd);} last_tti = tti;};

  /* Activate / Disactivate SCell*/
  void set_activation_deactivation_scell(uint32_t cmd, uint32_t tti) final {  phy_h->set_activation_deactivation_scell(cmd, tti); };

  /* Sets RAR dci payload */
  void set_rar_grant(uint8_t grant_payload[SRSRAN_RAR_GRANT_LEN], uint16_t rnti) final {  phy_h->set_rar_grant(grant_payload, rnti, index); };

  uint32_t get_current_tti() final {  return phy_h->get_current_tti(); };

  float get_phr()         final {  return phy_h->get_phr(); };
  float get_pathloss_db() final {  return phy_h->get_pathloss_db(); };

  void
  prach_send(uint32_t preamble_idx, int allowed_subframe, float target_power_dbm, float ta_base_sec = 0.0f) final {  
    printf("Stack %d is listening for new connections...\n", index);
    phy_h->prach_send(preamble_idx, allowed_subframe, target_power_dbm, ta_base_sec); };
  phy_interface_mac_lte::prach_info_t prach_get_info()                                                                     final {  return phy_h->prach_get_info(); };

  /* Indicates the transmission of a SR signal in the next opportunity */
  void sr_send()        final {  phy_h->sr_send(); };
  int  sr_last_tx_tti() final {  return phy_h->sr_last_tx_tti(); };

  void set_mch_period_stop(uint32_t stop) final {  phy_h->set_mch_period_stop(stop); };

  // RRC -> PHY

  bool set_config(const srsran::phy_cfg_t& config, uint32_t cc_idx = 0)           final {  return phy_h->set_config(config, cc_idx = 0, index); };
  bool set_scell(srsran_cell_t cell_info, uint32_t cc_idx, uint32_t earfcn)       final {  return false; };//return phy_h->set_scell(cell_info, cc_idx, earfcn); }; -> scells for now not available
  void set_config_tdd(srsran_tdd_config_t& tdd_config)                            final {  phy_h->set_config_tdd(tdd_config); };
  void set_config_mbsfn_sib2(srsran::mbsfn_sf_cfg_t* cfg_list, uint32_t nof_cfgs) final {  phy_h->set_config_mbsfn_sib2(cfg_list, nof_cfgs); };
  void set_config_mbsfn_sib13(const srsran::sib13_t& sib13)                       final {  phy_h->set_config_mbsfn_sib13(sib13); };
  void set_config_mbsfn_mcch(const srsran::mcch_msg_t& mcch)                      final {  phy_h->set_config_mbsfn_mcch(mcch); };

  void deactivate_scells() final {  phy_h->deactivate_scells(); };

  /* Measurements interface */
  void set_cells_to_meas(uint32_t earfcn, const std::set<uint32_t>& pci) final {  phy_h->set_cells_to_meas(earfcn, pci); };
  void meas_stop()                                                       final {  phy_h->meas_stop(); };

  /* Cell search and selection procedures */
  bool cell_search()                final {  return phy_h->cell_search(index); };
  bool cell_select(phy_cell_t cell) final {  return phy_h->cell_select(cell, index); };
  bool cell_is_camping()            final {  return phy_h->cell_is_camping(); };

private:
  phy* phy_h;
  int  index;
  uint32_t last_tti;
};

} // namespace srsue

#endif // SRSUE_PHY_PROXY_H
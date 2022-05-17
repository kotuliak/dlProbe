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

#include "srsran/common/bearer_manager.h"

namespace srsran {

namespace detail {

const ue_bearer_manager_impl::radio_bearer_t ue_bearer_manager_impl::invalid_rb{srsran::srsran_rat_t::nulltype, 0, 0};

bool ue_bearer_manager_impl::add_eps_bearer(uint8_t eps_bearer_id, srsran::srsran_rat_t rat, uint32_t lcid)
{
  auto bearer_it = bearers.find(eps_bearer_id);
  if (bearer_it != bearers.end()) {
    return false;
  }
  bearers.emplace(eps_bearer_id, radio_bearer_t{rat, lcid, eps_bearer_id});
  lcid_to_eps_bearer_id.emplace(lcid, eps_bearer_id);
  return true;
}

bool ue_bearer_manager_impl::remove_eps_bearer(uint8_t eps_bearer_id)
{
  auto bearer_it = bearers.find(eps_bearer_id);
  if (bearer_it == bearers.end()) {
    return false;
  }
  uint32_t lcid = bearer_it->second.lcid;
  bearers.erase(bearer_it);
  lcid_to_eps_bearer_id.erase(lcid);
  return true;
}

void ue_bearer_manager_impl::reset()
{
  lcid_to_eps_bearer_id.clear();
  bearers.clear();
}

bool ue_bearer_manager_impl::has_active_radio_bearer(uint32_t eps_bearer_id)
{
  return bearers.count(eps_bearer_id) > 0;
}

ue_bearer_manager_impl::radio_bearer_t ue_bearer_manager_impl::get_radio_bearer(uint32_t eps_bearer_id)
{
  auto it = bearers.find(eps_bearer_id);
  return it != bearers.end() ? it->second : invalid_rb;
}

ue_bearer_manager_impl::radio_bearer_t ue_bearer_manager_impl::get_eps_bearer_id_for_lcid(uint32_t lcid)
{
  auto lcid_it = lcid_to_eps_bearer_id.find(lcid);
  return lcid_it != lcid_to_eps_bearer_id.end() ? bearers.at(lcid_it->second) : invalid_rb;
}

} // namespace detail
} // namespace srsran

namespace srsue {

ue_bearer_manager::ue_bearer_manager() : logger(srslog::fetch_basic_logger("STCK", false))
{
  pthread_rwlock_init(&rwlock, nullptr);
}

ue_bearer_manager::~ue_bearer_manager()
{
  pthread_rwlock_destroy(&rwlock);
}

void ue_bearer_manager::add_eps_bearer(uint8_t eps_bearer_id, srsran::srsran_rat_t rat, uint32_t lcid)
{
  srsran::rwlock_write_guard rw_lock(rwlock);
  if (impl.add_eps_bearer(eps_bearer_id, rat, lcid)) {
    logger.info(
        "Bearers: Registered EPS bearer ID %d for lcid=%d over %s-PDCP", eps_bearer_id, lcid, to_string(rat).c_str());
  } else {
    logger.error("Bearers: EPS bearer ID %d already registered", eps_bearer_id);
  }
}

void ue_bearer_manager::remove_eps_bearer(uint8_t eps_bearer_id)
{
  srsran::rwlock_write_guard rw_lock(rwlock);
  if (impl.remove_eps_bearer(eps_bearer_id)) {
    logger.info("Bearers: Removed mapping for EPS bearer ID %d", eps_bearer_id);
  } else {
    logger.error("Bearers: Can't remove EPS bearer ID %d", eps_bearer_id);
  }
}

void ue_bearer_manager::reset()
{
  srsran::rwlock_write_guard rw_lock(rwlock);
  impl.reset();
  logger.info("Bearers: Reset EPS bearer manager");
}

} // namespace srsue
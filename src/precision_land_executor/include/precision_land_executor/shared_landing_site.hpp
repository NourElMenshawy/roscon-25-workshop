#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace precision_land_executor {

struct Point3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Site {
  std::string id;
  Point3 center;
  double clearance_radius_m{0.0};
  bool verified{false};
  bool occupied{false};
};

struct SiteCatalog {
  std::uint64_t mission_id{0};
  std::string owner_id;
  std::string frame_id;
  std::vector<Site> sites;
};

// Created only from a successful tf2 lookup for this specific site and mission.
// The pure eligibility policy does not depend on ROS 2 or perform transforms.
struct ResolvedSite {
  std::uint64_t mission_id{0};
  std::string site_id;
  std::string source_frame;
  std::string target_frame;
  Point3 local_position;
};

enum class SiteRejection : std::uint8_t {
  Eligible,
  MissingCatalog,
  WrongMission,
  MissingOccupiedSite,
  MissingCandidate,
  UnverifiedSite,
  InvalidFrame,
  InvalidGeometry,
  InsufficientSeparation,
};

struct SiteEvaluation {
  bool position_available{false};
  bool frame_valid{false};
  bool site_verified{false};
  SiteRejection reason{SiteRejection::MissingCatalog};
  std::string selected_site_id;
  Point3 local_position;
};

inline constexpr const char* toString(SiteRejection reason) noexcept {
  switch (reason) {
    case SiteRejection::Eligible: return "eligible";
    case SiteRejection::MissingCatalog: return "missing_catalog";
    case SiteRejection::WrongMission: return "wrong_mission";
    case SiteRejection::MissingOccupiedSite: return "missing_occupied_site";
    case SiteRejection::MissingCandidate: return "missing_candidate_site";
    case SiteRejection::UnverifiedSite: return "unverified_site";
    case SiteRejection::InvalidFrame: return "invalid_frame_or_transform";
    case SiteRejection::InvalidGeometry: return "invalid_geometry";
    case SiteRejection::InsufficientSeparation: return "insufficient_separation";
  }
  return "unknown";
}

inline bool finite(Point3 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// Applies mission, occupancy, physical-site and frame checks. A resolved point
// must be for the exact candidate, frames and mission; a stale TF result cannot
// authorize a new mission or a different site. The first pass may supply nullopt
// to validate geometry before attempting any TF lookup.
inline SiteEvaluation evaluateSites(const SiteCatalog* catalog,
                                    std::uint64_t mission_id,
                                    const std::string& primary_owner,
                                    const std::string& common_frame,
                                    const std::string& local_frame,
                                    const std::optional<ResolvedSite>& resolved,
                                    double separation_margin_m) {
  SiteEvaluation result{};
  if (catalog == nullptr) {
    return result;
  }
  if (catalog->mission_id != mission_id || mission_id == 0 ||
      catalog->owner_id != primary_owner || primary_owner.empty()) {
    result.reason = SiteRejection::WrongMission;
    return result;
  }

  const Site* occupied = nullptr;
  const Site* candidate = nullptr;
  for (const auto& site : catalog->sites) {
    if (site.occupied) {
      // There must be exactly one primary occupied site in this demo.
      if (occupied != nullptr) {
        result.reason = SiteRejection::InvalidGeometry;
        return result;
      }
      occupied = &site;
    } else {
      // This milestone supports one unoccupied candidate, not ranking sites.
      if (candidate != nullptr) {
        result.reason = SiteRejection::InvalidGeometry;
        return result;
      }
      candidate = &site;
    }
  }
  if (occupied == nullptr) {
    result.reason = SiteRejection::MissingOccupiedSite;
    return result;
  }
  if (candidate == nullptr) {
    result.reason = SiteRejection::MissingCandidate;
    return result;
  }
  if (occupied->id.empty() || candidate->id.empty() ||
      occupied->id == candidate->id) {
    result.reason = SiteRejection::InvalidGeometry;
    return result;
  }
  result.position_available = true;
  result.selected_site_id = candidate->id;

  if (common_frame.empty() || local_frame.empty() ||
      catalog->frame_id != common_frame) {
    result.reason = SiteRejection::InvalidFrame;
    return result;
  }
  if (!finite(occupied->center) || !finite(candidate->center) ||
      !std::isfinite(occupied->clearance_radius_m) ||
      !std::isfinite(candidate->clearance_radius_m) ||
      !std::isfinite(separation_margin_m) ||
      occupied->clearance_radius_m <= 0.0 ||
      candidate->clearance_radius_m <= 0.0 || separation_margin_m < 0.0) {
    result.reason = SiteRejection::InvalidGeometry;
    return result;
  }
  if (!occupied->verified || !candidate->verified) {
    result.reason = SiteRejection::UnverifiedSite;
    return result;
  }

  const double dx = occupied->center.x - candidate->center.x;
  const double dy = occupied->center.y - candidate->center.y;
  const double required = occupied->clearance_radius_m +
                          candidate->clearance_radius_m + separation_margin_m;
  if (std::hypot(dx, dy) < required) {
    result.reason = SiteRejection::InsufficientSeparation;
    return result;
  }

  // Only TF data for the exact mission, site and frame pair may be consumed.
  // No manually configured quaternion or translation is accepted here.
  if (!resolved || resolved->mission_id != mission_id ||
      resolved->site_id != candidate->id ||
      resolved->source_frame != common_frame ||
      resolved->target_frame != local_frame ||
      !finite(resolved->local_position)) {
    result.reason = SiteRejection::InvalidFrame;
    return result;
  }

  result.frame_valid = true;
  result.site_verified = true;
  result.local_position = resolved->local_position;
  result.reason = SiteRejection::Eligible;
  return result;
}

}  // namespace precision_land_executor

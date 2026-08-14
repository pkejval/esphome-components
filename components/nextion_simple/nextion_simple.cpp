// nextion_simple.cpp

#include "nextion_simple.h"

#include "esphome/core/log.h"
#include "esphome/core/util.h"

#if defined(USE_ESP32) && defined(USE_ESP_IDF)
#include "esp_heap_caps.h"
#endif

namespace esphome {
namespace nextion_simple {

const char *NextionSimple::TAG = "nextion_simple";

NextionSimple::NextionSimple() = default;

uint32_t NextionSimple::fnv1a32_(const char *s) {
  if (s == nullptr)
    return 0;
  uint32_t h = 2166136261u;
  for (; *s; s++) {
    h ^= static_cast<uint8_t>(*s);
    h *= 16777619u;
  }
  return h;
}

uint32_t NextionSimple::make_coalesce_key_(uint32_t comp_hash, TxCoalesceKind kind) {
  // Keep all hash bits. The component name and kind are compared as well
  // before two entries are coalesced, so even an FNV collision cannot merge
  // updates belonging to different components.
  return comp_hash ^ (static_cast<uint32_t>(kind) * 0x9e3779b9U);
}

namespace {

static uint32_t mix32_(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

}  // namespace

void NextionSimple::dump_config() {
  ESP_LOGCONFIG(TAG, "Nextion Simple:");
  ESP_LOGCONFIG(TAG, "  TFT URL: %s", this->tft_url_.c_str());
  ESP_LOGCONFIG(TAG, "  TX batching: queue=%u, max_per_loop=%u, budget=%" PRIu32 "us, loop budget=%" PRIu32 "us",
                 static_cast<unsigned>(TXQ_SIZE), static_cast<unsigned>(this->tx_max_per_loop_),
                 static_cast<uint32_t>(this->tx_time_budget_us_), static_cast<uint32_t>(this->loop_time_budget_us_));
}

void NextionSimple::setup() {
  ESP_LOGCONFIG(TAG, "Initializing Nextion Simple...");
  if (this->uart_parent_ == nullptr) {
    ESP_LOGE(TAG, "UART parent not set!");
    mark_failed();
    return;
  }

  this->start_init_handshake_();
  this->on_setup_callback_.call();
}

void NextionSimple::loop() {
  if (this->uart_parent_ == nullptr)
    return;

  // The normal write-only idle path is the overwhelmingly common case. Avoid
  // even reading the microsecond clock unless there is pending work.
  if (!this->upload_in_progress_ && this->mode_ == NxMode::RUN_WRITEONLY && !this->txq_has_raw_() &&
      !this->txm_has_dirty_())
    return;

  const uint32_t loop_deadline_us = micros() + this->loop_time_budget_us_;
  if (!this->upload_in_progress_) {
    this->tx_flush_(loop_deadline_us);
  }

  if (this->upload_in_progress_)
    return;

  switch (this->mode_) {
    case NxMode::INIT:
      this->init_tick_(loop_deadline_us);
      break;

    case NxMode::RUN_WRITEONLY:
      break;

    case NxMode::DIAG_CHECK:
      this->diagnostic_tick_(loop_deadline_us);
      break;
  }
}

// ================= RX =================

void NextionSimple::reset_rx_state_() {
  this->rb_head_ = 0;
  this->rb_tail_ = 0;
  this->rb_overflow_ = false;

  this->in_frame_ = false;
  this->frame_len_ = 0;
  this->ff_term_ = 0;
  this->frame_overflow_ = false;
  this->frame_start_ms_ = 0;
  this->frame_bytes_seen_ = 0;
}

static inline bool is_init_start(uint8_t b) { return b == 0x66 || b == 0x88 || b == 0x00; }
static inline bool is_diag_start(uint8_t b) { return b == 0x66 || b == 0x70 || b == 0x71; }

bool NextionSimple::rb_find_next_frame_start_(RxFilter filter, uint8_t &b, uint32_t deadline_us, uint8_t &parse_iters) {
  const size_t mask = RB_SIZE - 1;

  while (this->rb_head_ != this->rb_tail_) {
    const size_t chunk = (this->rb_head_ > this->rb_tail_) ? (this->rb_head_ - this->rb_tail_) : (RB_SIZE - this->rb_tail_);
    const uint8_t *src = &this->rx_rb_[this->rb_tail_];
    size_t consumed = 0;

    for (size_t i = 0; i < chunk; i++) {
      const uint8_t c = src[i];
      const bool start = (filter == RxFilter::INIT_ONLY) ? is_init_start(c) : is_diag_start(c);
      if (start) {
        this->rb_tail_ = (this->rb_tail_ + i + 1) & mask;
        b = c;
        return true;
      }

      consumed = i + 1;

      if ((++parse_iters & 0x0F) == 0 && static_cast<int32_t>(micros() - deadline_us) >= 0) {
        this->rb_tail_ = (this->rb_tail_ + consumed) & mask;
        return false;
      }
    }

    this->rb_tail_ = (this->rb_tail_ + chunk) & mask;
  }

  return false;
}

void NextionSimple::log_rx_drops_if_needed_() {
  const uint32_t now = millis();

  if (this->rx_rb_drop_count_ != 0 && (now - this->rx_rb_drop_last_log_ms_ >= this->rx_rb_drop_log_interval_ms_)) {
    ESP_LOGW(TAG, "RX ring dropped %" PRIu32 " byte(s) in recent interval", this->rx_rb_drop_count_);
    this->rx_rb_drop_count_ = 0;
    this->rx_rb_drop_last_log_ms_ = now;
    this->rx_rb_drop_log_interval_ms_ = this->rx_rb_drop_log_interval_ms_ < 60000 ? this->rx_rb_drop_log_interval_ms_ * 2 : 60000;
  }

  if (this->rx_frame_drop_count_ != 0 && (now - this->rx_frame_drop_last_log_ms_ >= this->rx_frame_drop_log_interval_ms_)) {
    ESP_LOGW(TAG, "Dropped %" PRIu32 " RX frame(s) due to overflow in recent interval", this->rx_frame_drop_count_);
    this->rx_frame_drop_count_ = 0;
    this->rx_frame_drop_last_log_ms_ = now;
    this->rx_frame_drop_log_interval_ms_ = this->rx_frame_drop_log_interval_ms_ < 60000 ? this->rx_frame_drop_log_interval_ms_ * 2 : 60000;
  }
}

void NextionSimple::drain_uart_into_ring_(uint32_t loop_deadline_us) {
  if (!this->rx_enabled_ || this->uart_parent_ == nullptr)
    return;
  if (static_cast<int32_t>(micros() - loop_deadline_us) >= 0)
    return;

  const uint32_t start_us = micros();
  uint32_t deadline_us = start_us + this->rx_time_budget_us_;
  if (static_cast<int32_t>(loop_deadline_us - deadline_us) < 0)
    deadline_us = loop_deadline_us;
  uint32_t drained = 0;
  uint8_t drain_iters = 0;

  while (this->uart_parent_->available() && drained < this->rx_drain_max_per_loop_) {
    if ((++drain_iters & 0x03) == 0 && static_cast<int32_t>(micros() - deadline_us) >= 0)
      break;

    size_t avail = static_cast<size_t>(this->uart_parent_->available());
    if (avail == 0)
      break;

    const size_t remaining = static_cast<size_t>(this->rx_drain_max_per_loop_ - drained);
    size_t to_read = avail;
    if (to_read > remaining)
      to_read = remaining;
    if (to_read > sizeof(this->rx_read_buf_))
      to_read = sizeof(this->rx_read_buf_);

    const size_t available_space = (this->rb_tail_ > this->rb_head_)
                                        ? (this->rb_tail_ - this->rb_head_ - 1)
                                        : (RB_SIZE - this->rb_head_ + this->rb_tail_ - 1);

    if (to_read <= 1) {
      uint8_t b;
      if (!this->uart_parent_->read_byte(&b))
        break;
      (void) this->rb_push_(b);
      drained++;
      continue;
    }

    if (to_read <= available_space) {
      const size_t till_end = RB_SIZE - this->rb_head_;
      if (to_read <= till_end) {
        if (!this->uart_parent_->read_array(&this->rx_rb_[this->rb_head_], to_read))
          break;
      } else {
        if (!this->uart_parent_->read_array(&this->rx_rb_[this->rb_head_], till_end))
          break;
        if (!this->uart_parent_->read_array(&this->rx_rb_[0], to_read - till_end))
          break;
      }

      this->rb_head_ = (this->rb_head_ + to_read) & (RB_SIZE - 1);
      drained += static_cast<uint32_t>(to_read);
      continue;
    }

    if (!this->uart_parent_->read_array(this->rx_read_buf_, to_read)) {
      uint8_t b;
      if (!this->uart_parent_->read_byte(&b))
        break;
      (void) this->rb_push_(b);
      drained++;
      continue;
    }

    for (size_t i = 0; i < to_read; i++) {
      (void) this->rb_push_(this->rx_read_buf_[i]);
    }

    drained += static_cast<uint32_t>(to_read);
  }

  this->log_rx_drops_if_needed_();
}

bool NextionSimple::parse_from_ring_(RxFilter filter, uint32_t loop_deadline_us) {
  const uint32_t start_us = micros();
  uint32_t deadline_us = start_us + this->rx_time_budget_us_;
  if (static_cast<int32_t>(loop_deadline_us - deadline_us) < 0)
    deadline_us = loop_deadline_us;
  const uint32_t start_ms = millis();
  uint8_t b;
  uint8_t parse_iters = 0;

  while (true) {
    if (!this->in_frame_) {
      if (!this->rb_find_next_frame_start_(filter, b, deadline_us, parse_iters))
        return false;

      this->in_frame_ = true;
      this->frame_buf_[0] = b;
      this->frame_len_ = 1;
      this->ff_term_ = 0;
      this->frame_overflow_ = false;
      this->frame_start_ms_ = start_ms;
      this->frame_bytes_seen_ = 1;
      continue;
    }

    if (!this->rb_pop_(b))
      return false;

    if ((++parse_iters & 0x0F) == 0 && static_cast<int32_t>(micros() - deadline_us) >= 0) {
      return false;
    }

    this->frame_bytes_seen_++;
    // Since this loop has a strict microsecond budget, millis() won't advance significantly.
    // Using start_ms avoids calling millis() for every parsed byte.
    if ((start_ms - this->frame_start_ms_) > this->frame_timeout_ms_ ||
        this->frame_bytes_seen_ > this->frame_max_bytes_seen_) {
      this->in_frame_ = false;
      this->frame_len_ = 0;
      this->ff_term_ = 0;
      this->frame_overflow_ = false;
      this->frame_start_ms_ = 0;
      this->frame_bytes_seen_ = 0;
      continue;
    }

    if (!this->frame_overflow_) {
      if (this->frame_len_ < FRAME_BUF_SIZE) {
        this->frame_buf_[this->frame_len_++] = b;
      } else {
        this->frame_overflow_ = true;
        this->rx_frame_drop_count_++;
      }
    }

    if (b == 0xFF) {
      if (++this->ff_term_ == 3) {
        if (!this->frame_overflow_) {
          const size_t len_no_term = (this->frame_len_ >= 3) ? (this->frame_len_ - 3) : 0;
          this->handle_frame_(filter, this->frame_buf_, len_no_term);
        }

        this->in_frame_ = false;
        this->frame_len_ = 0;
        this->ff_term_ = 0;
        this->frame_overflow_ = false;
        this->frame_start_ms_ = 0;
        this->frame_bytes_seen_ = 0;

        if ((filter == RxFilter::INIT_ONLY && this->handshake_done_) ||
            (filter == RxFilter::DIAG_ONLY && this->saw_expected_reply_)) {
          return true;
        }
      }
    } else {
      this->ff_term_ = 0;
    }
  }

  return false;
}

void NextionSimple::handle_frame_(RxFilter filter, const uint8_t *frame, size_t len_no_term) {
  if (len_no_term == 0 || frame == nullptr)
    return;

  const uint8_t code = frame[0];

  if (filter == RxFilter::DIAG_ONLY) {
    if (code == 0x70 || code == 0x71) {
      this->saw_expected_reply_ = true;
      return;
    }

    if (code == 0x66 && len_no_term >= 2) {
      this->current_page_ = static_cast<int>(frame[1]);
      this->on_page_callback_.call(this->current_page_);
      if (this->page_sync_active_ && this->current_page_ == this->page_sync_target_)
        this->saw_expected_reply_ = true;
    }
    return;
  }

  switch (code) {
    case 0x66: {
      if (len_no_term >= 2) {
        this->current_page_ = static_cast<int>(frame[1]);
        this->on_page_callback_.call(this->current_page_);
      }
      this->handshake_done_ = true;
      break;
    }
    case 0x88:
      break;
    case 0x00:
      ESP_LOGW(TAG, "Nextion returned invalid instruction during INIT");
      break;
    default:
      break;
  }
}

// ================= Modes =================

void NextionSimple::init_tick_(uint32_t loop_deadline_us) {
  const uint32_t now = millis();
  if (this->handshake_done_ || now >= this->init_deadline_ms_) {
    if (!this->handshake_done_) {
      ESP_LOGW(TAG, "Init handshake timeout, proceeding to write-only.");
    }
    this->enter_writeonly_mode_();
    return;
  }

  this->drain_uart_into_ring_(loop_deadline_us);
  if (static_cast<int32_t>(micros() - loop_deadline_us) < 0)
    (void) this->parse_from_ring_(RxFilter::INIT_ONLY, loop_deadline_us);

  if (this->handshake_done_ || millis() >= this->init_deadline_ms_) {
    if (!this->handshake_done_) {
      ESP_LOGW(TAG, "Init handshake timeout, proceeding to write-only.");
    }
    this->enter_writeonly_mode_();
  }
}

void NextionSimple::start_init_handshake_() {
  this->mode_ = NxMode::INIT;
  this->rx_enabled_ = true;
  this->handshake_done_ = false;
  this->saw_expected_reply_ = false;
  this->reset_rx_state_();
  this->init_deadline_ms_ = millis() + 3000;

  if (this->bkcmd_ != 3) {
    this->send_command_printf("bkcmd=3");
    this->bkcmd_ = 3;
  }
  this->send_command_printf("sendme");
}

void NextionSimple::enter_writeonly_mode_() {
  if (this->bkcmd_ != 0) {
    this->send_command_printf("bkcmd=0");
    this->bkcmd_ = 0;
  }

  this->rx_enabled_ = false;
  this->reset_rx_state_();
  this->mode_ = NxMode::RUN_WRITEONLY;

  const uint32_t now = millis();
  if (now - this->last_ready_ms_ >= this->nextion_ready_cooldown_) {
    this->last_ready_ms_ = now;
    this->on_nextion_ready_callback_.call();
  }
}

void NextionSimple::request_health_check_() {
  if (this->mode_ != NxMode::RUN_WRITEONLY)
    return;
  if (this->upload_in_progress_ || this->uart_parent_ == nullptr)
    return;

  if (this->bkcmd_ != 1) {
    this->send_command_printf("bkcmd=1");
    this->bkcmd_ = 1;
  }

  this->rx_enabled_ = true;
  this->saw_expected_reply_ = false;
  this->reset_rx_state_();
  this->diag_deadline_ms_ = millis() + this->diag_timeout_ms_;
  this->mode_ = NxMode::DIAG_CHECK;

  this->send_command_printf("get dim");
}

void NextionSimple::diagnostic_tick_(uint32_t loop_deadline_us) {
  if (!this->rx_enabled_) {
    this->mode_ = NxMode::RUN_WRITEONLY;
    return;
  }

  this->drain_uart_into_ring_(loop_deadline_us);
  if (static_cast<int32_t>(micros() - loop_deadline_us) < 0)
    (void) this->parse_from_ring_(RxFilter::DIAG_ONLY, loop_deadline_us);

  if (this->page_sync_active_) {
    if (this->saw_expected_reply_ || this->current_page_ == this->page_sync_target_) {
      this->page_sync_active_ = false;
      this->page_sync_target_ = -1;
      this->page_sync_attempts_left_ = 0;
      if (this->bkcmd_ != 0) {
        this->send_command_printf("bkcmd=0");
        this->bkcmd_ = 0;
      }
      this->rx_enabled_ = false;
      this->reset_rx_state_();
      this->mode_ = NxMode::RUN_WRITEONLY;
      return;
    }

    if (millis() >= this->diag_deadline_ms_) {
      if (this->page_sync_attempts_left_ == 0 || this->page_sync_attempts_left_ > 1) {
        if (this->page_sync_attempts_left_ > 1) {
          this->page_sync_attempts_left_--;
        }
        ESP_LOGW(TAG, "Nextion page %d not confirmed, retrying (%u attempt(s) left)",
                 this->page_sync_target_, static_cast<unsigned>(this->page_sync_attempts_left_));
        this->saw_expected_reply_ = false;
        this->reset_rx_state_();
        this->diag_deadline_ms_ = millis() + this->diag_timeout_ms_;
        this->send_command_printf("page %d", this->page_sync_target_);
        this->send_command_printf("sendme");
        return;
      }

      ESP_LOGW(TAG, "Nextion page %d not confirmed after %u attempt(s)",
               this->page_sync_target_, static_cast<unsigned>(this->page_sync_attempts_left_));
      if (this->bkcmd_ != 0) {
        this->send_command_printf("bkcmd=0");
        this->bkcmd_ = 0;
      }
      this->page_sync_active_ = false;
      this->page_sync_target_ = -1;
      this->page_sync_attempts_left_ = 0;
      this->rx_enabled_ = false;
      this->reset_rx_state_();
      this->mode_ = NxMode::RUN_WRITEONLY;
      return;
    }

    return;
  }

  if (this->saw_expected_reply_ || millis() >= this->diag_deadline_ms_) {
    if (this->bkcmd_ != 0) {
      this->send_command_printf("bkcmd=0");
      this->bkcmd_ = 0;
    }
    this->rx_enabled_ = false;
    this->reset_rx_state_();
    this->mode_ = NxMode::RUN_WRITEONLY;
  }
}

// ================= TX queue (raw barriers + mirrored coalesced state) =================

bool NextionSimple::txq_push_raw_barrier_(const uint8_t *data, size_t len) {
  if (data == nullptr || len == 0)
    return false;

  // Raw command is an ordering barrier between mirrored epochs.
  this->tx_epoch_++;

  size_t nh = (this->txq_head_ + 1) & (TXQ_SIZE - 1);
  // Once overflow is in use, keep appending there until it has drained. This
  // preserves the ordering of older overflow commands ahead of new commands.
  if (!this->txq_overflow_.empty() || nh == this->txq_tail_) {
    TxEntry entry{};
    if (len > sizeof(entry.data))
      len = sizeof(entry.data);
    memcpy(entry.data, data, len);
    entry.len = static_cast<uint16_t>(len);
    entry.epoch = this->tx_epoch_;
    this->txq_overflow_.push_back(std::move(entry));
    return true;
  }

  auto &slot = this->txq_[this->txq_head_];
  if (len > sizeof(slot.data))
    len = sizeof(slot.data);
  memcpy(slot.data, data, len);

  slot.len = static_cast<uint16_t>(len);
  slot.epoch = this->tx_epoch_;

  this->txq_head_ = nh;
  return true;
}

const NextionSimple::TxEntry *NextionSimple::txq_peek_() {
  if (this->txq_head_ != this->txq_tail_)
    return &this->txq_[this->txq_tail_];
  if (!this->txq_overflow_.empty())
    return &this->txq_overflow_.front();
  return nullptr;
}

void NextionSimple::txq_consume_() {
  if (this->txq_head_ != this->txq_tail_) {
    this->txq_tail_ = (this->txq_tail_ + 1) & (TXQ_SIZE - 1);
  } else if (!this->txq_overflow_.empty()) {
    this->txq_overflow_.pop_front();
  }
}

NextionSimple::TxMirrorEntry *NextionSimple::txm_find_or_alloc_(uint32_t key, const char *component_name,
                                                                 TxMirrorKind kind) {
  if (key == 0 || component_name == nullptr)
    return nullptr;

  for (auto *entry : this->txm_lookup_cache_) {
    if (entry != nullptr && entry->used && entry->key == key && entry->kind == kind &&
        entry->component == component_name)
      return entry;
  }

  constexpr size_t mask = TX_MIRROR_SIZE - 1;
  size_t idx = mix32_(key) & mask;
  size_t step = (mix32_(key ^ 0x9e3779b9U) | 1U) & mask;
  size_t first_free = TX_MIRROR_SIZE;
  size_t first_clean = TX_MIRROR_SIZE;

  for (size_t i = 0; i < TX_MIRROR_SIZE; i++) {
    auto &e = this->txm_[idx];
    if (e.used && e.key == key && e.kind == kind && e.component == component_name) {
      this->txm_cache_(e);
      return &e;
    }
    if (!e.used && first_free == TX_MIRROR_SIZE)
      first_free = idx;
    if (e.used && !e.dirty && first_clean == TX_MIRROR_SIZE)
      first_clean = idx;
    idx = (idx + step) & mask;
  }

  if (first_free != TX_MIRROR_SIZE) {
    auto &e = this->txm_[first_free];
    this->txm_reuse_(e, key, false);
    this->txm_cache_(e);
    return &e;
  }

  for (auto &e : this->txm_overflow_) {
    if (e.key == key && e.kind == kind && e.component == component_name) {
      this->txm_cache_(e);
      return &e;
    }
  }

  // Prefer reusing a clean slot. Dirty entries may not be evicted: each is
  // an update that has not reached the display yet.
  if (first_clean != TX_MIRROR_SIZE) {
    auto &e = this->txm_[first_clean];
    this->txm_reuse_(e, key, false);
    this->txm_cache_(e);
    return &e;
  }

  for (auto &e : this->txm_overflow_) {
    if (!e.dirty) {
      this->txm_reuse_(e, key, true);
      this->txm_cache_(e);
      return &e;
    }
  }

  // A burst with more than TX_MIRROR_SIZE distinct dirty properties is rare,
  // but must not overwrite another component. This allocation happens only
  // in that overload path; the normal path remains allocation-free here.
  this->txm_overflow_.emplace_back();
  auto &e = this->txm_overflow_.back();
  this->txm_reuse_(e, key, true);
  this->txm_cache_(e);
  return &e;
}

void NextionSimple::txm_reuse_(TxMirrorEntry &entry, uint32_t key, bool overflow_storage) {
  // clear() keeps the allocated capacity on the supported ESP toolchains.
  // Reusing it avoids heap churn when a component slot is recycled.
  entry.used = true;
  entry.dirty = false;
  entry.overflow_storage = overflow_storage;
  entry.key = key;
  entry.epoch = this->tx_epoch_;
  entry.kind = TxMirrorKind::NONE;
  entry.cmd_len = 0;
  entry.component.clear();
  entry.prop.clear();
  entry.text.clear();
  entry.int_value = 0;
  entry.dirty_prev = nullptr;
  entry.dirty_next = nullptr;
}

void NextionSimple::txm_cache_(TxMirrorEntry &entry) {
  this->txm_lookup_cache_[this->txm_lookup_cache_next_] = &entry;
  this->txm_lookup_cache_next_ = (this->txm_lookup_cache_next_ + 1) & (TXM_LOOKUP_CACHE_SIZE - 1);
}

void NextionSimple::txm_mark_dirty_(TxMirrorEntry &entry, bool move_to_tail) {
  if (entry.dirty) {
    if (!move_to_tail)
      return;

    if (entry.dirty_prev != nullptr)
      entry.dirty_prev->dirty_next = entry.dirty_next;
    else
      this->dirty_head_ = entry.dirty_next;
    if (entry.dirty_next != nullptr)
      entry.dirty_next->dirty_prev = entry.dirty_prev;
    else
      this->dirty_tail_ = entry.dirty_prev;
  } else {
    entry.dirty = true;
  }

  entry.dirty_prev = this->dirty_tail_;
  entry.dirty_next = nullptr;
  if (this->dirty_tail_ != nullptr)
    this->dirty_tail_->dirty_next = &entry;
  else
    this->dirty_head_ = &entry;
  this->dirty_tail_ = &entry;
}

void NextionSimple::txm_clear_dirty_(TxMirrorEntry &entry) {
  if (!entry.dirty)
    return;

  if (entry.dirty_prev != nullptr)
    entry.dirty_prev->dirty_next = entry.dirty_next;
  else
    this->dirty_head_ = entry.dirty_next;
  if (entry.dirty_next != nullptr)
    entry.dirty_next->dirty_prev = entry.dirty_prev;
  else
    this->dirty_tail_ = entry.dirty_prev;

  entry.dirty = false;
  entry.dirty_prev = nullptr;
  entry.dirty_next = nullptr;
}



bool NextionSimple::txm_build_command_(TxMirrorEntry &e) {
  e.cmd_len = 0;

  this->tx_begin_();
  bool ok = true;

  switch (e.kind) {
    case TxMirrorKind::PROP_INT:
      ok &= this->tx_append_str_(e.component);
      ok &= this->tx_append_char_('.');
      ok &= this->tx_append_str_(e.prop);
      ok &= this->tx_append_char_('=');
      ok &= this->tx_append_int_(e.int_value);
      break;

    case TxMirrorKind::VIS:
      ok &= this->tx_append_cstr_("vis ");
      ok &= this->tx_append_str_(e.component);
      ok &= this->tx_append_char_(',');
      ok &= this->tx_append_int_(e.int_value);
      break;

    case TxMirrorKind::TXT:
      ok &= this->tx_append_str_(e.component);
      ok &= this->tx_append_cstr_(".txt=\"");
      ok &= this->tx_append_escaped_nextion_string_(e.text);
      ok &= this->tx_append_char_('"');
      break;

    default:
      return false;
  }

  if (!ok) {
    e.cmd_len = 0;
    return false;
  }

  this->tx_buf_[this->tx_len_] = 0xFF;
  this->tx_buf_[this->tx_len_ + 1] = 0xFF;
  this->tx_buf_[this->tx_len_ + 2] = 0xFF;

  e.cmd_len = static_cast<uint16_t>(this->tx_len_ + 3);
  memcpy(e.cmd_buf, this->tx_buf_, e.cmd_len);
  return true;
}

bool NextionSimple::txm_set_prop_int_(const char *component_name, const char *prop, TxCoalesceKind kind, int value) {
  if (component_name == nullptr || *component_name == '\0')
    return false;
  const char *prop_text = prop == nullptr ? "" : prop;
  const uint32_t key = make_coalesce_key_(fnv1a32_(component_name), kind);
  auto *e = this->txm_find_or_alloc_(key, component_name, TxMirrorKind::PROP_INT);
  if (e == nullptr)
    return false;

  const bool data_changed = e->kind != TxMirrorKind::PROP_INT || e->component != component_name || e->prop != prop_text ||
                            e->int_value != value;
  const bool epoch_changed = e->epoch != this->tx_epoch_;

  if (data_changed) {
    e->kind = TxMirrorKind::PROP_INT;
    e->component = component_name;
    e->prop = prop_text;
    e->int_value = value;
  }

  if (data_changed) {
    e->cmd_len = 0;
  }

  if (data_changed || epoch_changed) {
    e->epoch = this->tx_epoch_;
    this->txm_mark_dirty_(*e, epoch_changed);
  }

  return true;
}

bool NextionSimple::txm_set_vis_(const char *component_name, int state) {
  if (component_name == nullptr || *component_name == '\0')
    return false;
  const uint32_t key = make_coalesce_key_(fnv1a32_(component_name), TxCoalesceKind::VIS);
  auto *e = this->txm_find_or_alloc_(key, component_name, TxMirrorKind::VIS);
  if (e == nullptr)
    return false;

  const bool data_changed = e->kind != TxMirrorKind::VIS || e->component != component_name || e->int_value != state;
  const bool epoch_changed = e->epoch != this->tx_epoch_;

  if (data_changed) {
    e->kind = TxMirrorKind::VIS;
    e->component = component_name;
    e->prop.clear();
    e->text.clear();
    e->int_value = state;
  }

  if (data_changed) {
    e->cmd_len = 0;
  }

  if (data_changed || epoch_changed) {
    e->epoch = this->tx_epoch_;
    this->txm_mark_dirty_(*e, epoch_changed);
  }

  return true;
}

bool NextionSimple::txm_set_text_(const char *component_name, const char *text) {
  if (component_name == nullptr || *component_name == '\0')
    return false;
  const char *text_ptr = text == nullptr ? "" : text;
  const uint32_t key = make_coalesce_key_(fnv1a32_(component_name), TxCoalesceKind::TXT);
  auto *e = this->txm_find_or_alloc_(key, component_name, TxMirrorKind::TXT);
  if (e == nullptr)
    return false;

  const bool data_changed = e->kind != TxMirrorKind::TXT || e->component != component_name || e->text != text_ptr;
  const bool epoch_changed = e->epoch != this->tx_epoch_;

  if (data_changed) {
    e->kind = TxMirrorKind::TXT;
    e->component = component_name;
    e->prop.clear();
    e->text = text_ptr;
  }

  if (data_changed) {
    e->cmd_len = 0;
  }

  if (data_changed || epoch_changed) {
    e->epoch = this->tx_epoch_;
    this->txm_mark_dirty_(*e, epoch_changed);
  }

  return true;
}

void NextionSimple::tx_flush_(uint32_t loop_deadline_us) {
  if (this->uart_parent_ == nullptr)
    return;

  // Extremely fast idle path without calling micros()
  if (!this->txq_has_raw_() && !this->txm_has_dirty_()) {
    return;
  }

  const uint32_t now = micros();
  if (this->uart_clear_micros_ != 0 && static_cast<int32_t>(now - this->uart_clear_micros_) < 0) {
    // Hardware is still transmitting physical layer bytes using Baud limitation - Yield!
    return;
  }

  // Do not increase this limit for a backlog.  A burst may improve display
  // latency, but it monopolizes the ESP loop exactly when the system is busy.
  // Coalesced state will still converge to the newest value on later passes.
  const uint8_t max_per_loop = this->tx_max_per_loop_;
  uint32_t deadline_us = now + this->tx_time_budget_us_;
  if (loop_deadline_us != 0 && static_cast<int32_t>(loop_deadline_us - deadline_us) < 0)
    deadline_us = loop_deadline_us;
  uint8_t sent = 0;
  uint32_t bytes_sent_this_tick = 0;

  while (sent < max_per_loop) {
    if (static_cast<int32_t>(micros() - deadline_us) >= 0)
      break;

    const bool has_dirty = this->txm_has_dirty_();
    const bool has_raw = this->txq_has_raw_();

    if (!has_dirty && !has_raw)
      break;

    TxMirrorEntry *dirty = this->dirty_head_;
    const TxEntry *head_raw = has_raw ? this->txq_peek_() : nullptr;
    const bool send_dirty = dirty != nullptr && (head_raw == nullptr || dirty->epoch < head_raw->epoch);

    if (send_dirty) {
      if (dirty->cmd_len == 0 && !this->txm_build_command_(*dirty)) {
        this->txm_clear_dirty_(*dirty);
        continue;
      }
      if (sent != 0 && bytes_sent_this_tick + dirty->cmd_len > this->tx_max_bytes_per_loop_)
        break;
      this->uart_parent_->write_array(dirty->cmd_buf, dirty->cmd_len);
      bytes_sent_this_tick += static_cast<uint32_t>(dirty->cmd_len);
      this->txm_clear_dirty_(*dirty);
      sent++;
      continue;
    }

    const TxEntry *raw = this->txq_peek_();
    if (raw == nullptr)
      continue;

    if (sent != 0 && bytes_sent_this_tick + raw->len > this->tx_max_bytes_per_loop_)
      break;
    this->uart_parent_->write_array(raw->data, raw->len);
    bytes_sent_this_tick += static_cast<uint32_t>(raw->len);
    this->txq_consume_();
    sent++;
  }

  if (bytes_sent_this_tick > 0) {
    const uint32_t baud = this->uart_parent_->get_baud_rate();
    // (1 start + 8 data + 1 stop bit) = 10 bits per byte. 
    // Time in microseconds = (bytes * 10 * 1000000) / baud
    const uint32_t transmit_time_us = (bytes_sent_this_tick * 10ULL * 1000000ULL) / baud;
    this->uart_clear_micros_ = micros() + transmit_time_us;
  }
}

// ================= TX builders =================

void NextionSimple::tx_begin_() { this->tx_len_ = 0; }

bool NextionSimple::tx_append_char_(char c) {
  if (this->tx_len_ >= kMaxCmd)
    return false;
  this->tx_buf_[this->tx_len_++] = static_cast<uint8_t>(c);
  return true;
}

bool NextionSimple::tx_append_cstr_(const char *s) {
  if (s == nullptr)
    return false;
  while (*s) {
    if (!tx_append_char_(*s++))
      return false;
  }
  return true;
}

bool NextionSimple::tx_append_str_(const std::string &s) {
  const size_t n = s.size();
  if (this->tx_len_ + n > kMaxCmd)
    return false;
  memcpy(this->tx_buf_ + this->tx_len_, s.data(), n);
  this->tx_len_ += n;
  return true;
}

bool NextionSimple::tx_append_int_(int v) {
  if (this->tx_len_ >= kMaxCmd)
    return false;

  uint32_t x;
  if (v < 0) {
    if (!tx_append_char_('-'))
      return false;
    x = static_cast<uint32_t>(-(int64_t) v);
  } else {
    x = static_cast<uint32_t>(v);
  }

  char tmp[12];
  int i = 0;
  do {
    tmp[i++] = static_cast<char>('0' + (x % 10));
    x /= 10;
  } while (x != 0 && i < static_cast<int>(sizeof(tmp)));

  while (i--) {
    if (!tx_append_char_(tmp[i]))
      return false;
  }
  return true;
}

void NextionSimple::tx_send_raw_barrier_() {
  if (this->uart_parent_ == nullptr)
    return;

  this->tx_buf_[this->tx_len_] = 0xFF;
  this->tx_buf_[this->tx_len_ + 1] = 0xFF;
  this->tx_buf_[this->tx_len_ + 2] = 0xFF;

  const size_t n = this->tx_len_ + 3;
  (void) this->txq_push_raw_barrier_(this->tx_buf_, n);
}

bool NextionSimple::tx_append_escaped_nextion_string_(const char *s) {
  if (s == nullptr)
    return false;

  for (; *s; s++) {
    const char c = *s;
    switch (c) {
      case '\\':
        if (!this->tx_append_char_('\\') || !this->tx_append_char_('\\'))
          return false;
        break;
      case '"':
        if (!this->tx_append_char_('\\') || !this->tx_append_char_('"'))
          return false;
        break;
      case '\r':
      case '\n':
      case '\t':
        if (!this->tx_append_char_(' '))
          return false;
        break;
      default:
        if (!this->tx_append_char_(c))
          return false;
        break;
    }
  }
  return true;
}

bool NextionSimple::tx_append_escaped_nextion_string_(const std::string &s) {
  return this->tx_append_escaped_nextion_string_(s.c_str());
}

void NextionSimple::send_prop_int_(const char *component_name, const char *prop, TxCoalesceKind kind, int value) {
  if (this->uart_parent_ == nullptr)
    return;
  (void) this->txm_set_prop_int_(component_name, prop, kind, value);
}

void NextionSimple::send_vis_(const char *component_name, int state) {
  if (this->uart_parent_ == nullptr)
    return;
  (void) this->txm_set_vis_(component_name, state);
}

void NextionSimple::send_set_text_(const char *component_name, const char *text) {
  if (this->uart_parent_ == nullptr)
    return;
  if (text == nullptr)
    return;
  (void) this->txm_set_text_(component_name, text);
}

void NextionSimple::send_set_text_formatted_(const std::string &component_name, const std::string &fmt,
                                             const std::vector<std::string> &args) {
  if (this->uart_parent_ == nullptr)
    return;

  char text[kMaxCmd + 1]{};
  size_t text_len = 0;
  size_t arg_i = 0;
  const char *p = fmt.c_str();
  bool overflow = false;

  auto append_char = [&](char c) -> bool {
    if (text_len >= kMaxCmd)
      return false;
    text[text_len++] = c;
    text[text_len] = '\0';
    return true;
  };

  auto append_str = [&](const std::string &s) -> bool {
    for (char c : s) {
      if (!append_char(c))
        return false;
    }
    return true;
  };

  while (*p) {
    if (p[0] == '%' && p[1] == 's' && arg_i < args.size()) {
      if (!append_str(args[arg_i++]))
        overflow = true;
      p += 2;
      if (overflow)
        break;
      continue;
    }

    if (!append_char(*p++))
      overflow = true;
    if (overflow)
      break;
  }

  if (overflow)
    return;

  (void) this->txm_set_text_(component_name.c_str(), text);
}

// ================= High-level API =================

void NextionSimple::set_component_value(const char *component_name, float value) {
  this->send_prop_int_(component_name, "val", TxCoalesceKind::VAL, static_cast<int>(value));
}

void NextionSimple::set_component_value(const std::string &component_name, float value) {
  this->set_component_value(component_name.c_str(), value);
}

void NextionSimple::set_component_text(const char *component_name, const char *text) {
  this->send_set_text_(component_name, text);
}

void NextionSimple::set_component_text(const char *component_name, const std::string &text) {
  this->send_set_text_(component_name, text.c_str());
}

void NextionSimple::set_component_text(const std::string &component_name, const std::string &text,
                                        const std::vector<std::string> &args) {
  if (args.empty()) {
    this->send_set_text_(component_name.c_str(), text.c_str());
    return;
  }
  this->send_set_text_formatted_(component_name, text, args);
}

void NextionSimple::set_component_text_printf(const char *component_name, const char *format, ...) {
  if (component_name == nullptr || format == nullptr)
    return;

  va_list ap;
  va_start(ap, format);
  char text[kMaxCmd + 1]{};
  const int tn = vsnprintf(text, sizeof(text), format, ap);
  va_end(ap);

  if (tn <= 0 || tn >= static_cast<int>(sizeof(text)))
    return;

  this->send_set_text_(component_name, text);
}

void NextionSimple::set_component_text_printf(const std::string &component_name, const char *format, ...) {
  if (format == nullptr)
    return;

  va_list ap;
  va_start(ap, format);
  char text[kMaxCmd + 1]{};
  const int tn = vsnprintf(text, sizeof(text), format, ap);
  va_end(ap);

  if (tn <= 0 || tn >= static_cast<int>(sizeof(text)))
    return;

  this->send_set_text_(component_name.c_str(), text);
}

void NextionSimple::set_component_picc(const char *component_name, int value) {
  this->send_prop_int_(component_name, "picc", TxCoalesceKind::PICC, value);
}

void NextionSimple::set_component_picc(const std::string &component_name, int value) {
  this->set_component_picc(component_name.c_str(), value);
}

void NextionSimple::set_component_picc1(const char *component_name, int value) {
  this->send_prop_int_(component_name, "picc1", TxCoalesceKind::PICC1, value);
}

void NextionSimple::set_component_picc1(const std::string &component_name, int value) {
  this->set_component_picc1(component_name.c_str(), value);
}

void NextionSimple::set_component_background_color(const char *component_name, int color) {
  this->send_prop_int_(component_name, "bco", TxCoalesceKind::BCO, color);
}

void NextionSimple::set_component_background_color(const std::string &component_name, int color) {
  this->set_component_background_color(component_name.c_str(), color);
}

void NextionSimple::set_component_background_color(const std::string &component_name, Color color) {
  this->set_component_background_color(component_name, this->color_to_integer_(color));
}

void NextionSimple::set_component_font_color(const char *component_name, int color) {
  this->send_prop_int_(component_name, "pco", TxCoalesceKind::PCO, color);
}

void NextionSimple::set_component_font_color(const std::string &component_name, int color) {
  this->set_component_font_color(component_name.c_str(), color);
}

void NextionSimple::set_component_font_color(const std::string &component_name, Color color) {
  this->set_component_font_color(component_name, this->color_to_integer_(color));
}

void NextionSimple::set_component_visibility(const char *component_name, bool state) {
  this->set_component_visibility(component_name, state ? 1 : 0);
}

void NextionSimple::set_component_visibility(const char *component_name, int state) {
  this->send_vis_(component_name, state);
}

void NextionSimple::set_component_visibility(const std::string &component_name, bool state) {
  this->set_component_visibility(component_name.c_str(), state ? 1 : 0);
}

void NextionSimple::set_component_visibility(const std::string &component_name, int state) {
  this->set_component_visibility(component_name.c_str(), state);
}

void NextionSimple::set_page(int page) {
  if (page < 0)
    return;
  this->send_command_printf("page %d", page);
  this->current_page_ = page;
  this->on_page_callback_.call(page);
}

void NextionSimple::set_page(const std::string &page_name) {
  if (page_name.empty())
    return;
  this->send_command_printf("page %s", page_name.c_str());
  this->current_page_ = -1;
  this->on_page_callback_.call(-1);
}

void NextionSimple::set_page_verified(int page, uint8_t retries, uint32_t verify_timeout_ms) {
  if (page < 0 || this->uart_parent_ == nullptr || this->upload_in_progress_)
    return;

  this->page_sync_active_ = true;
  this->page_sync_target_ = page;
  this->page_sync_attempts_left_ = retries;
  this->diag_timeout_ms_ = verify_timeout_ms;
  this->saw_expected_reply_ = false;
  this->rx_enabled_ = true;
  this->reset_rx_state_();
  this->diag_deadline_ms_ = millis() + this->diag_timeout_ms_;
  this->mode_ = NxMode::DIAG_CHECK;

  this->send_command_printf("page %d", page);
  this->send_command_printf("sendme");
}

// ================= Low-level (raw barrier enqueue) =================

void NextionSimple::send_command(const char *cmd, size_t len) {
  if (this->uart_parent_ == nullptr || cmd == nullptr)
    return;

  if (len > kMaxCmd)
    len = kMaxCmd;

  memcpy(this->tx_buf_, cmd, len);
  this->tx_buf_[len] = 0xFF;
  this->tx_buf_[len + 1] = 0xFF;
  this->tx_buf_[len + 2] = 0xFF;

  (void) this->txq_push_raw_barrier_(this->tx_buf_, len + 3);
}

void NextionSimple::send_command_printf(const char *fmt, ...) {
  if (this->uart_parent_ == nullptr || fmt == nullptr)
    return;

  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(reinterpret_cast<char *>(this->tx_buf_), kMaxCmd, fmt, ap);
  va_end(ap);

  if (n <= 0)
    return;

  if (n >= static_cast<int>(kMaxCmd)) {
    const uint32_t now = millis();
    if (now - this->tx_trunc_last_log_ms_ >= 1000) {
      ESP_LOGW(TAG, "TX command truncated to %u bytes", static_cast<unsigned>(kMaxCmd));
      this->tx_trunc_last_log_ms_ = now;
    }
  }

  size_t len = static_cast<size_t>(n);
  if (len > kMaxCmd)
    len = kMaxCmd;

  this->tx_buf_[len] = 0xFF;
  this->tx_buf_[len + 1] = 0xFF;
  this->tx_buf_[len + 2] = 0xFF;

  (void) this->txq_push_raw_barrier_(this->tx_buf_, len + 3);
}

// ================= Maintenance =================

void NextionSimple::reset_nextion() {
  this->send_command_printf("rest");
}

void NextionSimple::upload_tft() {
  if (this->tft_url_.empty()) {
    ESP_LOGW(TAG, "TFT URL not configured; skipping upload");
    return;
  }
  if (this->upload_in_progress_)
    return;

  // Flush queue before upload
  this->tx_flush_();

  this->upload_in_progress_ = true;
#if defined(USE_ESP_IDF)
  (void) this->upload_tft_esp_idf_();
#else
  (void) this->upload_tft_arduino_();
#endif
}

uint32_t NextionSimple::get_free_heap_() {
#if defined(USE_ESP32) && defined(USE_ESP_IDF)
  return esp_get_free_heap_size();
#elif defined(USE_ESP32)
  return ESP.getFreeHeap();
#elif defined(USE_ESP8266)
  return ESP.getFreeHeap();
#else
  return 0;
#endif
}

}  // namespace nextion_simple
}  // namespace esphome

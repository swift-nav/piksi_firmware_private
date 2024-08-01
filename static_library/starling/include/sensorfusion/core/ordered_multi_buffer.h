///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
///////////////////////////////////////////////////////////////////////////////

#ifndef SENSORFUSION_CORE_ORDERED_MULTIBUFFER_H_
#define SENSORFUSION_CORE_ORDERED_MULTIBUFFER_H_

#include <assert.h>

#include <array>
#include <functional>
#include <optional.hpp>
#include <tuple>
#include <utility>

#include "pvt_common/containers/circular_buffer.h"
#include "pvt_common/containers/static_vector.h"
#include "sensorfusion/core/error_types.h"
#include "sensorfusion/core/ordered_multi_buffer_internal.h"
#include "sensorfusion/core/traits.h"

namespace sensorfusion {

//! Heterogeneous container ordered by common key. Intended for realtime
//! buffering and playback of sequential, heterogeneous data (sensor readings).
//!
//! Implemented as a collection of circular-buffers, this class is supposed
//! to provide fast write access, along with the possibility for simultaneous
//! playback over multiple "playback heads". Data is flushed after it has
//! been considered by all active playback heads.
//!
//! If you are familiar with the \c std::variant / \c std::visit combination
//! then you will recognize that this class is comparable to a variant buffer
//! which supports multiple simultaneous visitors.
//!
//! The following terminology is used when describing this class:
//! \li \c Key used to determine ordering across heterogenous types
//! \li \c Stream refers to inbound data sequence of singular type
//! \li \c Buffer one buffer per inbound stream
//! \li \c Reader visitor class which will process heterogeneous data in
//! sequence
//!
//! @tparam Key Used to determine playback ordering.
//!  Must support relational operators.
//!  Copied frequently so smaller is faster.
//!
//! @tparam Streams Pack of unique stream declarations.
template <class Key, class... Streams>
class OrderedMultiBuffer
    : omb_internal::OrderedMultiBufferBase<Key, Streams...> {
 public:
  //! @brief Optional data type.
  template <class T>
  using optional = std::experimental::optional<T>;

  //! @brief Value type of a specific stream.
  template <class Stream>
  using stream_value_t = typename Stream::value_type;

  //! @brief Strongly-typed handle used to read buffered elements.
  template <class... ReaderStreams>
  using ReaderHandle = omb_internal::ReaderHandle<ReaderStreams...>;

  //! @brief Return type when creating handles.
  template <class... ReaderStreams>
  using MaybeReaderHandle = ValueOrError<ReaderHandle<ReaderStreams...>>;

  //--------------------------------------------------------------------------
  //! @name Readers
  //!
  //! Methods for reading the data stored in the buffers. Clients should
  //! register a handle which represents a read pointer into the buffer.
  //! Such handles may optionally reference some subset of the streams
  //! which are currently being buffered.
  //!@{

  //! @brief Create a new read handle for the requested streams.
  //! @tparam ReaderStreams Subset of stream types which the handle reads.
  template <class... ReaderStreams>
  MaybeReaderHandle<ReaderStreams...> create_reader_handle(
      ReaderStreams &&... /*tags*/);

  //! @brief Reset the reader state associated to the given handle
  template <class... ReaderStreams>
  void reset_reader_handle(const ReaderHandle<ReaderStreams...> &handle);

  //! @brief Use a registered handle to read data up until a target key.
  template <class Reader, class... ReaderStreams>
  void read_until_no_flush(const Key &target,
                           const ReaderHandle<ReaderStreams...> &handle,
                           Reader &&reader);

  //! @brief Use a registered handle to read data up until a target key.
  template <class Reader, class... ReaderStreams>
  void read_until(const Key &target,
                  const ReaderHandle<ReaderStreams...> &handle,
                  Reader &&reader);

  //!@}

  //--------------------------------------------------------------------------
  //! @name Modifiers
  //!
  //! Methods for modifying the data stored in the buffers.
  //!@{

  //! @brief Write a key/value pair into buffer for the indicated stream.
  template <class Stream>
  MaybeError buffer_write(Stream && /*tag*/, const Key &key,
                          const stream_value_t<Stream> &value);

  //! @brief Clear buffer for the indicated stream.
  template <class Stream>
  void buffer_clear(Stream && /*tag*/);

  //! @brief Clear all buffered data.
  void clear();

  //! @brief Flush data which has been consumed by all registered readers.
  void flush_obsolete_data();

  ///@}

  //--------------------------------------------------------------------------
  //! @name Capacity
  //!
  //! Methods relating to element storage.
  //!@{

  //! @brief Compile-time capacity of buffer for indicated stream.
  template <class Stream>
  static constexpr size_t buffer_capacity(Stream && /*tag*/);

  //! @brief Current number of elements in buffer for indicated stream.
  template <class Stream>
  size_t buffer_count(Stream && /*tag*/) const;

  //! @brief True if buffer for indicated stream is empty.
  template <class Stream>
  bool buffer_empty(Stream && /*tag*/) const;

  //! @brief Total number of elements stored across all constituent buffers.
  size_t count() const;

  //! @brief True if all constituent buffers are empty.
  bool empty() const;

  //!@}

  //!--------------------------------------------------------------------------
  //! @name Keys
  //!
  //! Methods for accessing information about buffered keys.
  //!@{

  //! @brief Smallest key in buffer for indicated stream.
  template <class Stream>
  optional<Key> buffer_min_key(Stream && /*tag*/) const;

  //! @brief Largest key in buffer for indicated stream.
  template <class Stream>
  optional<Key> buffer_max_key(Stream && /*tag*/) const;

  //! @brief Smallest key across all buffers.
  optional<Key> min_key() const;

  //! @brief Largest key across all buffers.
  optional<Key> max_key() const;

  //!@}

 private:
  //----------------------------------------------------------------------------
  // SFINAE trait-checkers specific to this module.
  //----------------------------------------------------------------------------

  // Check for the method T.process_data(...).
  template <class T, class S, class = void>
  struct has_method_process_data : std::false_type {};
  template <class T, class S>
  struct has_method_process_data<T, S,
                                 decltype(std::declval<T>().process_data(
                                     std::declval<S>(),
                                     std::declval<const Key>(),
                                     std::declval<const stream_value_t<S>>()))>
      : std::true_type {};

  // Check for the method T.handle_dropped_data(...).
  template <class T, class S, class = void>
  struct has_method_handle_dropped_data : std::false_type {};
  template <class T, class S>
  struct has_method_handle_dropped_data<
      T, S, decltype(std::declval<T>().handle_dropped_data(std::declval<S>()))>
      : std::true_type {};

  //----------------------------------------------------------------------------
  // Type aliases (mostly for convenience).
  //----------------------------------------------------------------------------

  // Total number of data streams in this multi-buffer.
  static constexpr size_t kNumStreams = traits::pack_size_v<Streams...>;

  // Alias for converting type into internal index.
  template <class Stream>
  static constexpr size_t stream_index_v =
      traits::pack_index_v<Stream, Streams...>;

  // Alias for checking that a reader can process data from streams.
  template <class Reader, class... ReaderStreams>
  static constexpr bool reader_can_process_data_v = traits::boolean_all_v<
      has_method_process_data<Reader, ReaderStreams>::value...>;

  // Alias for checking that a reader can handle when streams drop data.
  template <class Reader, class... ReaderStreams>
  static constexpr bool reader_can_handle_dropped_data_v =
      traits::boolean_all_v<
          has_method_handle_dropped_data<Reader, ReaderStreams>::value...>;

  template <class T, size_t N>
  using StaticVector = pvt_common::containers::StaticVector<T, N>;

  template <class T, size_t N>
  using CircularBuffer = pvt_common::containers::CircularBuffer<T, N>;

  //----------------------------------------------------------------------------
  // Implementation types.
  //----------------------------------------------------------------------------

  // Type used for buffering Key/Value pairs for each stream.
  template <class Stream>
  struct Item {
    Key key;
    stream_value_t<Stream> value;
  };

  // Each stream is granted a circular buffer object for storing data.
  using CircularBufferTuple =
      std::tuple<CircularBuffer<Item<Streams>, Streams::required_capacity>...>;

  // Effectively a "variant" type used when dispatching the next
  // item in sequence to readers.
  class TypeErasedItemRef {
   public:
    TypeErasedItemRef() : buffer_index_{kNumStreams}, key_{}, value_{nullptr} {}

    template <class Stream>
    TypeErasedItemRef(const Item<Stream> &item)
        : buffer_index_{stream_index_v<Stream>},
          key_{item.key},
          value_{&item.value} {}

    bool has_value() const { return key_.has_value(); }

    const Key &key() const {
      assert(has_value());
      return key_.value();
    }

    const void *value() const {
      assert(has_value());
      return value_;
    }

    size_t buffer_index() const { return buffer_index_; }

   private:
    optional<Key> key_;
    const void *value_;
    size_t buffer_index_;
  };

  // Type-erased dispatcher function.
  template <class Reader>
  using DispatchFunction = void (*)(Reader &, const TypeErasedItemRef &);

  // Dispatch method for ignored types.
  struct NullDispatch {
    template <class Reader, class>
    static constexpr DispatchFunction<Reader> method = nullptr;
  };

  // Dispatch method for required types.
  struct UserDispatch {
    template <class Reader, class Stream>
    static void method(Reader &reader, const TypeErasedItemRef &item_ref) {
      using Value = stream_value_t<Stream>;
      const Value &v = *reinterpret_cast<const Value *>(item_ref.value());
      reader.process_data(Stream{}, item_ref.key(), v);
    }
  };

  // State encapsulation for a single "reader". Maintains offsets
  // into each circular buffer tracking the number of outstanding
  // values which still need to be read.
  class ReaderState {
    using Bufs = CircularBufferTuple;

   public:
    ReaderState()
        : prev_key_{},
          num_unread_{0},
          is_buffer_active_{((void)Streams{}, false)...},
          was_data_dropped_bit_{((void)Streams{}, false)...} {}

    template <class... ReaderStreams>
    ReaderState(const Bufs &bufs, ReaderStreams &&... /*streams*/)
        : prev_key_{},
          num_unread_{static_cast<size_t>(
              std::get<stream_index_v<Streams>>(bufs).size())...},
          is_buffer_active_{traits::is_in_pack_v<Streams, ReaderStreams...>...},
          was_data_dropped_bit_{((void)Streams{}, false)...} {}

    void reset() {
      for (size_t i = 0; i < kNumStreams; ++i) {
        num_unread_[i] = 0;
      }
    }

    template <class Stream>
    void reset_stream(Stream &&stream) {
      num_unread_[stream_index_v<Stream>] = 0;
    }

    template <class... ReaderStreams>
    bool has_more(ReaderStreams &&... /*streams*/) const {
      static_assert(traits::pack_size_v<ReaderStreams...>> 0, "");
      for (const size_t i : {stream_index_v<ReaderStreams>...}) {
        if (0 != num_unread_[i]) {
          return true;
        }
      }
      return false;
    }

    template <class Stream>
    bool is_buffer_active(Stream && /*stream*/) const {
      return is_buffer_active_[stream_index_v<Stream>];
    }

    template <class Stream>
    size_t num_unread(Stream && /*stream*/) const {
      constexpr size_t required_capacity = Stream::required_capacity;
      return std::min(num_unread_[stream_index_v<Stream>], required_capacity);
    }

    template <class Reader, class... ReaderStreams>
    void handle_any_dropped_data(const Bufs &bufs, Reader &reader,
                                 ReaderStreams &&... /*streams*/) {
      static_assert(traits::pack_size_v<ReaderStreams...>> 0, "");
      int _[] = {
          (was_data_dropped(ReaderStreams{}, bufs)
               ? (reader.handle_dropped_data(ReaderStreams{}),
                  update_after_handled_dropped_data(ReaderStreams{}, bufs), 0)
               : 0)...};
      int __[] = {
          (was_data_dropped_bit_[stream_index_v<ReaderStreams>] = false, 0)...};
    }

    template <class Stream>
    TypeErasedItemRef ref_to_next_buffered_item(Stream && /*tag*/,
                                                const Bufs &bufs) const {
      if (!has_more(Stream{})) {
        return {};
      }
      constexpr size_t i = stream_index_v<Stream>;
      return {*(std::get<i>(bufs).cend() - num_unread_[i])};
    }

    template <class Stream>
    void update_after_item_write(Stream && /*stream*/, const Bufs &bufs,
                                 const Item<Stream> &item) {
      constexpr size_t i = stream_index_v<Stream>;
      const bool is_read_needed = (!prev_key_ || (item.key >= *prev_key_));
      if (is_read_needed &&
          num_unread_[i] < std::numeric_limits<size_t>::max()) {
        num_unread_[i]++;
      }
      const bool is_too_late = (prev_key_ && item.key < *prev_key_);
      if (is_too_late) {
        was_data_dropped_bit_[i] = true;
      }
    }

    void update_after_item_read(const TypeErasedItemRef &item_ref) {
      prev_key_ = item_ref.key();
      if (num_unread_[item_ref.buffer_index()] > 0) {
        num_unread_[item_ref.buffer_index()]--;
      }
    }

   private:
    optional<Key> prev_key_;
    size_t num_unread_[kNumStreams];
    bool is_buffer_active_[kNumStreams];
    bool was_data_dropped_bit_[kNumStreams];

    template <class Stream>
    bool was_data_dropped(Stream && /*stream*/, const Bufs &bufs) const {
      constexpr size_t i = stream_index_v<Stream>;
      const bool was_read_too_late = num_unread_[i] > std::get<i>(bufs).size();
      const bool was_write_too_late = was_data_dropped_bit_[i];
      return was_read_too_late || was_write_too_late;
    }

    template <class Stream>
    void update_after_handled_dropped_data(Stream && /*stream*/,
                                           const Bufs &bufs) {
      constexpr size_t i = stream_index_v<Stream>;
      const size_t count = std::get<i>(bufs).size();
      assert(was_data_dropped(Stream{}, bufs));
      if (num_unread_[i] > std::get<i>(bufs).size()) {
        num_unread_[i] = count;
      }
    }
  };

  //----------------------------------------------------------------------------
  // Helper methods.
  //----------------------------------------------------------------------------

  // For a single stream, pop any elements which have been consumed by
  // all readers which care about this stream.
  template <class Stream>
  void buffer_flush_obsolete(Stream && /*tag*/) {
    constexpr size_t buffer_index = stream_index_v<Stream>;
    size_t largest_unread = 0;
    for (const auto &state : reader_states_) {
      if (state.is_buffer_active(Stream{})) {
        largest_unread = std::max(largest_unread, state.num_unread(Stream{}));
      }
    }
    assert(buffer_count(Stream{}) >= largest_unread);
    const size_t num_obsolete = buffer_count(Stream{}) - largest_unread;
    std::get<buffer_index>(buffers_).pop_earliest(num_obsolete);
  }

  // Write an item for a specific stream. Returns an error if the item
  // is out of order.
  template <class Stream>
  MaybeError buffer_write_item(const Item<Stream> &item) {
    constexpr size_t buffer_index = stream_index_v<Stream>;
    auto &buf = std::get<buffer_index>(buffers_);
    const auto prev_key = prev_keys_[buffer_index];
    // It is assumed that all individual buffer sequences are ordered.
    if (prev_key && prev_key.value() > item.key) {
      return Error("item sequence ordering was violated");
    }
    prev_keys_[buffer_index] = item.key;
    buf.insert(item);
    return Ok();
  }

  // Given a set of streams, scan through the corresponding buffers to
  // find the next item. "Next" is defined by the item with the
  // minimum key across all buffers. When multiple items have the same
  // key, the stream occuring earlier in the parameter pack is given
  // precedence.
  template <class... ReaderStreams>
  TypeErasedItemRef scan_for_next_item(const ReaderState &state,
                                       ReaderStreams &&... /*tags*/) const {
    static constexpr size_t n = traits::pack_size_v<ReaderStreams...>;
    static_assert(n > 0, "");
    std::array<TypeErasedItemRef, n> next_item_refs = {
        state.ref_to_next_buffered_item(ReaderStreams{}, buffers_)...};
    // Linear scan for the "minimum" key.
    TypeErasedItemRef &best = next_item_refs[0];
    for (size_t i = 1; i < next_item_refs.size(); ++i) {
      const TypeErasedItemRef &item = next_item_refs[i];
      if (!best.has_value()) {
        best = item;
      } else {
        if (item.has_value() && item.key() < best.key()) {
          best = item;
        }
      }
    }
    return best;
  }

  // Type-safe dispatch a type-erased item to a given reader.
  // For each reader type, a static dispatch table is constructed,
  // which is then used to apply the proper forwarding method.
  // Because it is checked at compile time that the reader object
  // supports all required dispatch methods, it is guaranteed
  // that the runtime dispatch is valid.
  template <class Reader, class... ReaderStreams>
  void dispatch_item_to_reader(const TypeErasedItemRef &item_ref,
                               Reader &reader,
                               ReaderStreams &&... /*streams*/) {
    // Already checked, but here as an explicit precondition.
    static_assert(reader_can_process_data_v<Reader, ReaderStreams...>, "");
    static constexpr DispatchFunction<Reader> dispatch_table[] = {
        std::conditional_t<traits::is_in_pack_v<Streams, ReaderStreams...>,
                           UserDispatch,
                           NullDispatch>::template method<Reader, Streams>...};
    assert(nullptr != dispatch_table[item_ref.buffer_index()]);
    dispatch_table[item_ref.buffer_index()](reader, item_ref);
  }

  // Given a comparison operator and an array of optional keys,
  // choose the key which compares successfully against all others
  // in the array. In the case of a tie, keys occurring earlier in
  // the array are given preference. Empty optional only be returned
  // in the case where there were no non-empties in the input array.
  template <class Compare>
  optional<Key> superior_key(
      const std::array<optional<Key>, kNumStreams> &all_keys,
      Compare &&cmp) const {
    optional<Key> superior = {};
    for (auto &key : all_keys) {
      if (key) {
        if (!superior || cmp(*key, *superior)) {
          superior = *key;
        }
      }
    }
    return superior;
  }

 public:
  //----------------------------------------------------------------------------
  // Public interface.
  //----------------------------------------------------------------------------

  // Maximum number of readers supported by this implementation.
  static constexpr size_t kMaxNumReaders = 8;

 private:
  //--------------------------------------------------------------------------
  // Members.
  //--------------------------------------------------------------------------
  StaticVector<ReaderState, kMaxNumReaders> reader_states_;
  CircularBufferTuple buffers_;
  optional<Key> prev_keys_[kNumStreams];

//--------------------------------------------------------------------------
// For white-box unit testing.
#ifdef ORDERED_MULTI_BUFFER_FRIEND_CLASSNAME
  friend class ORDERED_MULTI_BUFFER_FRIEND_CLASSNAME;
#endif
};

//--------------------------------------------------------------------------
template <class Key, class... Streams>
template <class Stream>
constexpr size_t OrderedMultiBuffer<Key, Streams...>::buffer_capacity(
    Stream && /*tag*/) {
  return std::tuple_element_t<stream_index_v<Stream>,
                              CircularBufferTuple>::get_max_size();
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
template <class Stream>
size_t OrderedMultiBuffer<Key, Streams...>::buffer_count(
    Stream && /*tag*/) const {
  return std::get<stream_index_v<Stream>>(buffers_).size();
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
template <class Stream>
bool OrderedMultiBuffer<Key, Streams...>::buffer_empty(
    Stream && /*tag*/) const {
  return buffer_count(Stream{}) == 0;
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
size_t OrderedMultiBuffer<Key, Streams...>::count() const {
  size_t count = 0;
  int _[] = {(count += buffer_count(Streams{}), 0)...};
  return count;
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
bool OrderedMultiBuffer<Key, Streams...>::empty() const {
  bool is_empty = true;
  int _[] = {(is_empty = is_empty && buffer_empty(Streams{}), 0)...};
  return is_empty;
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
template <class Stream>
void OrderedMultiBuffer<Key, Streams...>::buffer_clear(Stream && /*tag*/) {
  for (auto &state : reader_states_) {
    state.reset_stream(Stream{});
  }
  std::get<stream_index_v<Stream>>(buffers_).clear();
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
template <class Stream>
MaybeError OrderedMultiBuffer<Key, Streams...>::buffer_write(
    Stream && /*tag*/, const Key &key, const stream_value_t<Stream> &value) {
  using Value = stream_value_t<Stream>;
  Item<Stream> item{key, value};
  auto result = buffer_write_item(item);
  if (result.is_ok()) {
    for (auto &state : reader_states_) {
      state.update_after_item_write(Stream{}, buffers_, item);
    }
  }
  return result;
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
template <class Stream>
optional<Key> OrderedMultiBuffer<Key, Streams...>::buffer_min_key(
    Stream && /*tag*/) const {
  if (buffer_count(Stream{}) == 0) {
    return {};
  }
  const auto first = std::get<stream_index_v<Stream>>(buffers_).cbegin();
  return first->key;
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
template <class Stream>
optional<Key> OrderedMultiBuffer<Key, Streams...>::buffer_max_key(
    Stream && /*tag*/) const {
  if (buffer_count(Stream{}) == 0) {
    return {};
  }
  const auto last = std::get<stream_index_v<Stream>>(buffers_).crbegin();
  return last->key;
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
void OrderedMultiBuffer<Key, Streams...>::clear() {
  int _[] = {(buffer_clear(Streams{}), 0)...};
  for (auto &state : reader_states_) {
    state.reset();
  }
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
void OrderedMultiBuffer<Key, Streams...>::flush_obsolete_data() {
  int _[] = {(buffer_flush_obsolete(Streams{}), 0)...};
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
optional<Key> OrderedMultiBuffer<Key, Streams...>::min_key() const {
  return superior_key({buffer_min_key(Streams{})...}, std::less<Key>{});
}

//--------------------------------------------------------------------------
template <class Key, class... Streams>
optional<Key> OrderedMultiBuffer<Key, Streams...>::max_key() const {
  return superior_key({buffer_max_key(Streams{})...}, std::greater<Key>{});
}

//--------------------------------------------------------------------------
// Doxygen does not recognize that this is the definition of a previously
// declared method.
#ifndef DOXYGEN_SHOULD_SKIP_THIS
template <class Key, class... Streams>
template <class... ReaderStreams>
ValueOrError<omb_internal::ReaderHandle<ReaderStreams...>>
OrderedMultiBuffer<Key, Streams...>::create_reader_handle(
    ReaderStreams &&... /*tags*/) {
  static_assert(traits::pack_size_v<ReaderStreams...>> 0,
                "cannot create reader handle for zero streams");
  static_assert(
      traits::boolean_all_v<traits::is_in_pack_v<ReaderStreams, Streams...>...>,
      "cannot create reader handle for unrecognized stream");
  static_assert(traits::are_types_unique_v<ReaderStreams...>,
                "cannot create reader handle with duplicate stream types");

  if (reader_states_.size() == kMaxNumReaders) {
    return Error("no more reader handles available");
  }
  size_t id = reader_states_.size();
  reader_states_.append(ReaderState{buffers_, ReaderStreams{}...});
  return ReaderHandle<ReaderStreams...>{id};
}
#endif

//--------------------------------------------------------------------------
//! @tparam Reader Object which supports the required \c process_data() methods.
//! @tparam ReaderStreams Streams referenced by this handle (and supported by
//! the reader).
//! @param target Target key which reading will proceed until.
//! @param handle Handle indicating which reader state is being accessed.
//! @param reader User "visitor" object which will react to data in the buffer.
//!
//! No data is removed from the buffers when using this function.
template <class Key, class... Streams>
template <class Reader, class... ReaderStreams>
void OrderedMultiBuffer<Key, Streams...>::read_until_no_flush(
    const Key &target, const ReaderHandle<ReaderStreams...> &handle,
    Reader &&reader) {
  static_assert(
      reader_can_handle_dropped_data_v<Reader, ReaderStreams...>,
      "cannot detect handle_dropped_data() method for requested streams");
  static_assert(reader_can_process_data_v<Reader, ReaderStreams...>,
                "cannot detect process_data() method for requested streams");

  assert(handle.id() < reader_states_.size());
  ReaderState &reader_state = reader_states_[handle.id()];

  reader_state.handle_any_dropped_data(buffers_, reader, ReaderStreams{}...);

  while (reader_state.has_more(ReaderStreams{}...)) {
    TypeErasedItemRef item_ref =
        scan_for_next_item(reader_state, ReaderStreams{}...);
    assert(item_ref.has_value() && "no loop body if buffers are exhausted");
    if (item_ref.key() <= target) {
      dispatch_item_to_reader(item_ref, reader, ReaderStreams{}...);
      reader_state.update_after_item_read(item_ref);
    } else {
      break;
    }
  }
}

//--------------------------------------------------------------------------
//! @see read_until_no_flush
//!
//! Any data which has been consumed by all active readers will be immediately
//! removed from the buffers.
template <class Key, class... Streams>
template <class Reader, class... ReaderStreams>
void OrderedMultiBuffer<Key, Streams...>::read_until(
    const Key &target, const ReaderHandle<ReaderStreams...> &handle,
    Reader &&reader) {
  read_until_no_flush(target, handle, std::forward<Reader>(reader));
  flush_obsolete_data();
}

template <class Key, class... Streams>
template <class... ReaderStreams>
void OrderedMultiBuffer<Key, Streams...>::reset_reader_handle(
    const ReaderHandle<ReaderStreams...> &handle) {
  ReaderState &reader_state = reader_states_[handle.id()];
  reader_state = ReaderState{buffers_, ReaderStreams{}...};
}

}  // namespace sensorfusion

#endif

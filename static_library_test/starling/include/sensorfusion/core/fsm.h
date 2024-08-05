//////////////////////////////////////////////////////////////////////////////
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

#ifndef SENSORFUSION_CORE_FSM_H_
#define SENSORFUSION_CORE_FSM_H_

//------------------------------------------------------------------------------
// About:
//
// Finite State Machine (almost). The main goal of this module is to provide
// relatively straightforward infrastructure for setting up well-defined
// transitions between processing modes and reacting to input events.
//
// You should find this helpful if you are looking to build a system with
// several processing states and you want to ensure that transition criteria
// between these states are clearly defined and well encapsulated. By using
// this module, you can benefit from the compiler enforcing that only valid
// state transitions are possible. You can also ensure that all events are
// properly handled in all states.
//
// References:
//
// Heavily inspired by the "tinyfsm" project, which is released
// under the MIT license by Axel Burri. This is a re-write mainly
// to avoid the third-party dependency for certification reasons,
// and also to tailor some of the specific behaviors for our use case.
//
//    https://github.com/digint/tinyfsm
//
//  One might quite fairly argue that Mr. Burri's version is much more
//  elegant than what is presented here, as tinyfsm requires no jump-tables.
//  The main advantage to the approach taken in this version is that there
//  is broader enforcement of FSM validity performed at compile time.
//
// Note:
//
// If you aren't currently in possession of some `C++ TMP Glasses™`, this can
// be a little tricky to read. Just keep in mind that everything in here is
// working towards building a set of runtime jump-tables which the compiler
// will have already checked for correctness.
//------------------------------------------------------------------------------

#include <array>
#include <cassert>
#include <tuple>
#include <type_traits>

#include "sensorfusion/core/error_types.h"
#include "sensorfusion/core/traits.h"
namespace sensorfusion {
namespace fsm {
namespace detail {

//------------------------------------------------------------------------------
// Check if a method "handle_event" exists on T, with return type R, and
// argument E.
//------------------------------------------------------------------------------
template <class T, class R, class E, class = R>
struct has_event_handler : std::false_type {};
template <class T, class R, class E>
struct has_event_handler<T, R, E,
                         decltype(std::declval<T>().handle_event(
                             std::declval<const E &>()))> : std::true_type {};
template <class T, class R, class E>
static constexpr bool has_event_handler_v = has_event_handler<T, R, E>::value;

//------------------------------------------------------------------------------
// Check if a method "enter" exists on T, with return type R, and no arguments.
//------------------------------------------------------------------------------
template <class T, class R, class = R>
struct has_method_enter : std::false_type {};
template <class T, class R>
struct has_method_enter<T, R, decltype(std::declval<T>().enter())>
    : std::true_type {};
template <class T, class R>
constexpr bool has_method_enter_v = has_method_enter<T, R>::value;

//------------------------------------------------------------------------------
// Check if a method "enter_from" exists on T, with return type R, and argument
// From.
//------------------------------------------------------------------------------
template <class T, class R, class From, class = R>
struct has_method_enter_from : std::false_type {};
template <class T, class R, class From>
struct has_method_enter_from<T, R, From,
                             decltype(std::declval<T>().enter_from(
                                 std::declval<const From &>()))>
    : std::true_type {};
template <class T, class R, class From>
constexpr bool has_method_enter_from_v =
    has_method_enter_from<T, R, From>::value;

//------------------------------------------------------------------------------
// Check for transition From -> To, with return type R.
//------------------------------------------------------------------------------
template <class R, class To, class From>
constexpr bool is_transition_supported_v =
    has_method_enter_v<To, R> || has_method_enter_from_v<To, R, From>;
}  // namespace detail

//------------------------------------------------------------------------------
// The structure here is a little odd, but it allows for defining an FSM with
// the syntax:
//
// using MyFsm = fsm::with_states<S1, S2, S3>::with_events<E1, E2>;
//------------------------------------------------------------------------------
template <class... States>
struct with_states {
  static_assert(traits::are_types_unique<States...>::value,
                "FSM_STATES_MUST_BE_UNIQUE");
  ~with_states() = delete;
  template <class... Events>
  struct with_events {
    static_assert(traits::are_types_unique<Events...>::value,
                  "FSM_EVENTS_MUST_BE_UNIQUE");
    ~with_events() = delete;
    //------------------------------------------------------------------------------
   private:
    using Definition = with_states<States...>::with_events<Events...>;
    using StateTuple = std::tuple<States...>;
    using EventTuple = std::tuple<Events...>;
    static constexpr size_t kNumStates = std::tuple_size<StateTuple>::value;
    static constexpr size_t kNumEvents = std::tuple_size<EventTuple>::value;

    // Check if a state comes from this definition. It is either:
    //  1. Part of the states parameter pack.
    //  2. A common base class of all types in the states parameter pack.
    template <class State>
    static constexpr bool is_valid_state_v =
        traits::is_in_tuple<StateTuple, State>::value;
    template <class State>
    static constexpr bool is_super_state_v =
        traits::boolean_all_v<std::is_base_of<State, States>::value...>;

    // Check if an event comes from this definition.
    template <class Event>
    static constexpr bool is_valid_event_v =
        traits::is_in_tuple<EventTuple, Event>::value;

    // Compile-time index for given state type.
    template <class State>
    static constexpr size_t state_index_v =
        traits::get_tuple_index<StateTuple, State>::value;
    static constexpr size_t STATE_UNKNOWN = kNumStates;

    // Compile-time index for given event type.
    template <class Event>
    static constexpr size_t event_index_v =
        traits::get_tuple_index<EventTuple, Event>::value;
    static constexpr size_t EVENT_UNKNOWN = kNumEvents;

    //------------------------------------------------------------------------------
   private:
    //----------------------------------------------------------------------------
    // "Control Flow":
    //
    // Object used to manage all control flow within an FSM instance. Client
    // software may only construct these objects via the static methods given
    // subsequently.
    //----------------------------------------------------------------------------
    class ControlFlowImpl {
     public:
      enum class Action {
        PANIC,
        CONTINUE,
        TRANSITION,
      };
      constexpr Action action() const { return action_; }
      constexpr size_t next() const { return next_; }

     protected:
      constexpr explicit ControlFlowImpl(Action action, size_t next)
          : action_(action), next_(next) {}

     private:
      Action action_ = Action::PANIC;
      size_t next_ = STATE_UNKNOWN;
    };

    //------------------------------------------------------------------------------
   public:
    //----------------------------------------------------------------------------
    // Public opaque control flow object.
    class ControlFlow : private ControlFlowImpl {
      using ControlFlowImpl::action;
      using ControlFlowImpl::ControlFlowImpl;
      using ControlFlowImpl::next;
      using typename ControlFlowImpl::Action;
      ControlFlow() = delete;
      friend Definition;
    };

    //----------------------------------------------------------------------------
    // PANIC is to be used when an unhandleable error has occurred and
    // the FSM should reinitialize, although the situation may be
    // unrecoverable.
    template <class Current>
    static constexpr ControlFlow Panic(Current *) {
      return ControlFlow{ControlFlow::Action::PANIC, STATE_UNKNOWN};
    }

    //----------------------------------------------------------------------------
    // CONTINUE indicates that the FSM should remain in its current state.
    template <class Current>
    static constexpr ControlFlow Continue(Current *) {
      return ControlFlow{ControlFlow::Action::CONTINUE, STATE_UNKNOWN};
    }

    //----------------------------------------------------------------------------
    // TRANSITION indicates a request for the FSM to transition into
    // another state. All compile-time enforcement of transition validity
    // is performed here.
    template <class Target, class Current>
    static constexpr ControlFlow TransitionTo(Current *) {
      static_assert(!std::is_same<Target, Current>::value,
                    "TRANSITION_MAY_ONLY_BE_USED_WITH_DISTINCT_STATES"
                    "(consider using 'continue')");
      static_assert(is_valid_state_v<Target>,
                    "TRANSITION_MAY_ONLY_TARGET_VALID_STATES");
      static_assert(
          detail::is_transition_supported_v<ControlFlow, Target, Current>,
          "UNSUPPORTED_TRANSITION");
      return ControlFlow{ControlFlow::Action::TRANSITION,
                         state_index_v<Target>};
    }

    //------------------------------------------------------------------------------
   private:
    //----------------------------------------------------------------------------
    // "State Pointer Access":
    //
    // Here, we build a table of lambda functions which can extract a raw
    // pointer to a tuple element. This is used to extract raw state pointers
    // for use when invoking all of the other jump-table functions.
    //----------------------------------------------------------------------------
    using StatePointerAccessor = void *(*)(StateTuple &states);

    template <class State>
    struct StatePointerAccessorDefinition {
      static constexpr void *pointer(StateTuple &states) {
        return std::addressof(std::get<state_index_v<State>>(states));
      }
    };

    using StatePointerAccessorTable =
        std::array<StatePointerAccessor, kNumStates>;

    static constexpr StatePointerAccessorTable
    make_state_pointer_accessor_table() {
      return {StatePointerAccessorDefinition<States>::pointer...};
    }

    //----------------------------------------------------------------------------
    // "Event Dispatch":
    //
    // Every state type must provide a dispatch method for each event type. A
    // jump-table is constructed with lambda functions which forward the event
    // into the corresponding state methods. The compiler will enforce that
    // every single cell in this table has a valid entry.
    //----------------------------------------------------------------------------
    using EventDispatchLambda = ControlFlow (*)(void *state, const void *event);

    template <class State, class Event>
    struct EventDispatchLambdaDefinition {
      static_assert(detail::has_event_handler_v<State, ControlFlow, Event>,
                    "STATE_HAS_NO_EVENT_HANDLER");
      static constexpr ControlFlow lambda(void *state, const void *event) {
        auto *pState = reinterpret_cast<State *>(state);
        auto *pEvent = reinterpret_cast<const Event *>(event);
        return pState->handle_event(*pEvent);
      }
    };

    using EventDispatchTableRow = std::array<EventDispatchLambda, kNumEvents>;
    using EventDispatchTable = std::array<EventDispatchTableRow, kNumStates>;

    template <class State>
    static constexpr EventDispatchTableRow make_event_dispatch_row() {
      return {EventDispatchLambdaDefinition<State, Events>::lambda...};
    }

    static constexpr EventDispatchTable make_event_dispatch_table() {
      return {make_event_dispatch_row<States>()...};
    }

    //----------------------------------------------------------------------------
    // "Transitions":
    //
    // It is common to store FSM transitions in a table. A jump-table is
    // constructed with lambda functions wrapping each supported transition. A
    // transition from state "S1" into state "S2" is considered supported if any
    // of the following criteria are satisfied:
    //
    //  * S2 implements specific entry method: "S2.enter_from(const S1&)"
    //  * S2 implements global entry method: "S2.enter()"
    //
    // Note that in the case where both criteria are satisfied, the jump-table
    // will always prefer the specific entry method over the global entry
    // method.
    //
    // Unsupported transitions are left NULL in the transition table, though
    // this is irrelevant as the compiler enforces that these unsupported
    // transitions are not accessed anywhere in the sofware.
    //----------------------------------------------------------------------------
    using TransitionLambda = ControlFlow (*)(const void *prev, void *next);

    template <class Prev, class Next, bool HasEnterFrom, bool HasEnter = false>
    struct TransitionLambdaDefinition {
      static_assert(HasEnterFrom,
                    "MOST_SPECIFIC_TRANSITION_REQUIRES_ENTER_FROM");
      static constexpr ControlFlow lambda(const void *prev, void *next) {
        auto *pPrev = reinterpret_cast<const Prev *>(prev);
        auto *pNext = reinterpret_cast<Next *>(next);
        return pNext->enter_from(*pPrev);
      }
    };

    template <class Prev, class Next>
    struct TransitionLambdaDefinition<Prev, Next, false, true> {
      static constexpr ControlFlow lambda(const void *, void *next) {
        auto *pNext = reinterpret_cast<Next *>(next);
        return pNext->enter();
      }
    };

    template <class Prev, class Next>
    struct TransitionLambdaDefinition<Prev, Next, false, false> {
      static constexpr TransitionLambda lambda = nullptr;
    };

    template <class Prev, class Next>
    static constexpr TransitionLambda make_transition_lambda() {
      return TransitionLambdaDefinition<
          Prev, Next, detail::has_method_enter_from_v<Next, ControlFlow, Prev>,
          detail::has_method_enter_v<Next, ControlFlow>>::lambda;
    }

    using TransitionTableRow = std::array<TransitionLambda, kNumStates>;
    using TransitionTable = std::array<TransitionTableRow, kNumStates>;

    template <class State>
    static constexpr TransitionTableRow make_transition_table_row() {
      return {make_transition_lambda<State, States>()...};
    }

    static constexpr TransitionTable make_transition_table() {
      return {make_transition_table_row<States>()...};
    }

    //----------------------------------------------------------------------------
    // "Global Entry":
    //
    // Some FSM operations require forcing into a certain state, regardless of
    // the origin state. The most obvious example of when this is required
    // would be "initializing". A jump-table is constructed with lambda
    // functions for all states which support a global entry method.
    //
    //  * A global entry method is of the form: "S.enter()"
    //
    // States which do not support global entry have a NULL entry in this table,
    // although this is irrelevant because the compiler enforces that global
    // entry can never be requested for a state which does not support it.
    //----------------------------------------------------------------------------
    using GlobalEntryLambda = ControlFlow (*)(void *state);

    template <class State, class HasEnter = void>
    struct GlobalEntryLambdaDefinition {
      static constexpr GlobalEntryLambda lambda = nullptr;
    };

    template <class State>
    struct GlobalEntryLambdaDefinition<
        State, std::enable_if_t<detail::has_method_enter_v<State, ControlFlow>,
                                void>> {
      static constexpr ControlFlow lambda(void *state) {
        return reinterpret_cast<State *>(state)->enter();
      }
    };

    using GlobalEntryTable = std::array<GlobalEntryLambda, kNumStates>;

    static constexpr GlobalEntryTable make_global_entry_table() {
      return {GlobalEntryLambdaDefinition<States>::lambda...};
    }

    //----------------------------------------------------------------------------
    // "FSM Instance":
    //
    // An actual object that can be used to satisfy all your FSM needs...
    //
    // Be careful to check the return codes on API functions which can error, as
    // this is the only way which a client is notified of runtime errors within
    // the state machine. Due to the large amount of compile-time enforcement
    // inherent in this design, there is a very small set of runtime errors
    // which can occur:
    //
    // * "panic" occurs when a state determines that an unrecoverable error has
    //   occurred. If you don't use states which PANIC, then this will never
    //   happen.
    //
    // * "loop" occurs when a potentially infinite loop is detected within the
    //   control flow. This can only occur if you have used states emit
    //   transitions from inside their entry methods.
    //
    // * "invalid" occurs when an attempt to use the API is made after either
    //   a "panic" or "loop" error has occurred, and the user has failed to
    //   successfully reinitialize the state machine. (this situation may also
    //   cause an assert failure).
    //----------------------------------------------------------------------------
    class InstanceImpl {
     public:
      //--------------------------------------------------------------------------
      // Return the state machine to its initial state.
      //
      // This function can error if the transition into the initial state
      // encounters either a LOOP or a PANIC.
      MaybeError init() { return force_into_state(starting_index_); }

      //--------------------------------------------------------------------------
      // Returns true if the request state is current in the FSM.
      template <class State>
      bool is_current_state() const {
        static_assert(is_valid_state_v<State>, "UNSUPPORTED_STATE");
        assert(is_in_valid_state());
        constexpr size_t state_index = state_index_v<State>;
        return state_index == current_index_;
      }

      //--------------------------------------------------------------------------
      // Returns an integral index representing the current state. The value is
      // guaranteed to be consistent with the order in which you defined the
      // FSM. Be careful when using this as all of the compile time checking
      // is pretty much thrown out the window. It can be useful in select
      // circumstances, but should not be used for control flow.
      size_t get_current_index() const {
        assert(is_in_valid_state());
        return current_index_;
      }

      //--------------------------------------------------------------------------
      // Returns a reference to the stored internal object of the given type.
      // In general you should prefer to interface with the state objects by
      // firing events into the FSM, but it can occasionally be useful to
      // interact directly with the internal objects. Just be aware that this
      // defeats the whole purpose of an FSM in the first place.
      template <class State>
      State &get_state_object() {
        static_assert(is_valid_state_v<State>, "UNSUPPORTED_STATE");
        constexpr size_t state_index = state_index_v<State>;
        return std::get<state_index>(states_);
      }

      //--------------------------------------------------------------------------
      // Returns a reference to the stored internal object of the given type
      // (const version)
      template <class State>
      const State &get_state_object() const {
        static_assert(is_valid_state_v<State>, "UNSUPPORTED_STATE");
        constexpr size_t state_index = state_index_v<State>;
        return std::get<state_index>(states_);
      }

      //--------------------------------------------------------------------------
      // Force the FSM to go to the desired state. Only supported for states
      // with global entry method.
      //
      // This function can error if the transition into the desired state
      // encounters either a LOOP or a PANIC.
      template <class State>
      MaybeError goto_state() {
        static_assert(is_valid_state_v<State>, "UNSUPPORTED_STATE");
        static_assert(detail::has_method_enter_v<State, ControlFlow>,
                      "GOTO_REQUIRES_STATE_WITH_GLOBAL_ENTER_METHOD");
        constexpr size_t goto_index = state_index_v<State>;
        return force_into_state(goto_index);
      }

      //--------------------------------------------------------------------------
      // Allow the FSM to react to an event. Behavior depends on how you
      // have implemented the event handlers in your state classes.
      //
      // This function can error if any resulting control flow results in
      // either a LOOP or a PANIC.
      template <class Event>
      MaybeError handle_event(const Event &event) {
        static_assert(is_valid_event_v<Event>, "UNSUPPORTED_EVENT");
        assert(is_in_valid_state());
        constexpr size_t event_index = event_index_v<Event>;
        auto &lambda = event_dispatch_table_[current_index_][event_index];
        void *pCurrent = get_state_pointer(current_index_);
        ControlFlow control = lambda(pCurrent, &event);
        return resolve_control_flow(control);
      }

      //----------------------------------------------------------------------------
     protected:
      template <class... Args>
      explicit InstanceImpl(size_t starting_index, Args &&... args)
          : starting_index_{starting_index},
            current_index_{STATE_UNKNOWN},
            states_{std::forward<Args>(args)...},
            state_pointer_accessor_table_{make_state_pointer_accessor_table()},
            event_dispatch_table_{make_event_dispatch_table()},
            global_entry_table_{make_global_entry_table()},
            transition_table_{make_transition_table()} {}

      //----------------------------------------------------------------------------
     private:
      const size_t starting_index_;
      size_t current_index_;
      StateTuple states_;
      StatePointerAccessorTable state_pointer_accessor_table_;
      EventDispatchTable event_dispatch_table_;
      GlobalEntryTable global_entry_table_;
      TransitionTable transition_table_;

      bool is_valid_index(size_t i) const { return (i < kNumStates); }

      bool is_in_valid_state() const { return is_valid_index(current_index_); }

      void *get_state_pointer(size_t state_index) {
        assert(is_valid_index(state_index));
        return state_pointer_accessor_table_[state_index](states_);
      }

      MaybeError force_into_state(const size_t state_index) {
        assert(is_valid_index(state_index));
        auto &lambda = global_entry_table_[state_index];
        assert(nullptr != lambda);
        current_index_ = state_index;
        void *pCurrent = get_state_pointer(current_index_);
        return resolve_control_flow(lambda(pCurrent));
      }

      MaybeError resolve_control_flow(ControlFlow control) {
        // Allow "sliding" between states until we reach a state which is happy
        // to continue itself. this means that ephemeral states are permitted,
        // but we must also detect cycles.
        int transition_count = 0;
        while (ControlFlow::Action::TRANSITION == control.action()) {
          const size_t next_index = control.next();
          assert(is_in_valid_state());
          assert(is_valid_index(next_index));
          void *pPrev = get_state_pointer(current_index_);
          void *pNext = get_state_pointer(next_index);
          auto &transit = transition_table_[current_index_][next_index];
          current_index_ = next_index;
          control = transit(pPrev, pNext);
          transition_count++;
          // Transitioning more times than the number of unique states means
          // that we are most likely beginning to loop indefinitely. if this
          // happens, the state machine is malformed.
          if (transition_count > kNumStates) {
            current_index_ = STATE_UNKNOWN;
            return Error("FSM transition loop detected");
          }
        }
        if (ControlFlow::Action::PANIC == control.action()) {
          current_index_ = STATE_UNKNOWN;
          return Error("FSM panic occurred");
        }
        return Ok();  // CONTINUE action was received.
      }
    };

    //------------------------------------------------------------------------------
   public:
    //----------------------------------------------------------------------------
    // Public opaque instance object.
    class Instance : private InstanceImpl {
     public:
      using InstanceImpl::get_current_index;
      using InstanceImpl::get_state_object;
      using InstanceImpl::goto_state;
      using InstanceImpl::handle_event;
      using InstanceImpl::init;
      using InstanceImpl::is_current_state;

     private:
      Instance() = delete;  // not strictly necessary, helps with error messages
      template <class... Args>
      explicit Instance(size_t starting_index, Args &&... args)
          : InstanceImpl(starting_index, std::forward<Args>(args)...) {}
      friend Definition;
    };

    //----------------------------------------------------------------------------
    // Builder used for creating FSM instances.
    template <class Initial>
    struct with_initial_state {
      static_assert(is_valid_state_v<Initial>,
                    "INITIALIZING_WITH_UNKNOWN_STATE");
      static_assert(detail::has_method_enter_v<Initial, ControlFlow>,
                    "INITIAL_STATE_MUST_HAVE_GLOBAL_ENTER_METHOD");
      ~with_initial_state() = delete;

      template <class... Args>
      static ValueOrError<Instance> create(Args &&... args) {
        constexpr size_t starting_index = state_index_v<Initial>;
        Instance instance{starting_index, std::forward<Args>(args)...};
        auto ret = instance.init();
        if (ret.is_error()) {
          return Error(ret.error_message());
        }
        return Ok<Instance>(instance);
      }
    };
  };
};

}  // namespace fsm
}  // namespace sensorfusion

#endif

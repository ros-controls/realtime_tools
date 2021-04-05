//
// Created by guru on 2/22/21.
//

#ifndef REALTIME_TOOLS__REALTIME_BARRIER_H
#define REALTIME_TOOLS__REALTIME_BARRIER_H

#include <mutex>
#include <thread>


namespace realtime_tools
{

///@brief Attempt a lock on a resource but fail if already locked
class try_lock : public std::unique_lock<std::mutex>
{
public:
  explicit try_lock(std::mutex & m)
  : std::unique_lock<std::mutex>(m, std::try_to_lock)
  {}
};

///@brief Lock a resource or wait for it to be free
class wait_lock : public std::unique_lock<std::mutex>
{
public:
  explicit wait_lock(std::mutex & m)
  : std::unique_lock<std::mutex>(m, std::defer_lock)
  {
    // will wait some time
    while (!try_lock()) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
};

///@brief default options for realtime acess to resources
struct realtime
{
  using lock_strategy = try_lock;
};

///@brief default options for non-realtime access to resources
struct non_realtime
{
  using lock_strategy = wait_lock;
};


///@brief Implements the underlying mechanisms for communicating between threads.
/// MemoryBarrier implements a double buffer swapping mechanism for interchanging data between
/// threads. It allocates two T objects on the heap. When MemoryBarrier::swap() is called the two
/// pointers are swapped. Generally one pointer is accessed from the non-realtime thread and the
/// other is used by the realtime thread. Only one-way communication, in either direction, is
/// possible. For two-way communication two separate MemoryBarriers can be used as a pair.
///
/// You should not use this class directly, you will work with ReadBarrier, WriteBarrier and
// DirectAccess classes only.
///
/// General strategy:
///   For non-RT writing to RT thread, non-RT thread writes but doesnt swap, RT swaps and reads.
///   For RT writing back to non-RT, RT writes but doesnt swap, NRT swaps and reads at will
template<class T>
class MemoryBarrier
{
public:
  ///@brief Provides direct access to data
  /// Should only be used from within the realtime thread. Default template args specify
  /// DirectAccess should only try to lock the resource and on failure the DirectAccess object
  /// would be equal to nullptr and therefore evaluate to false.
  /// This is also used internally by the non-RT ReadBarrier and WriteBarrier classes but
  /// implements the thread-safe access.
  template<typename access_mode = realtime,
    typename lock_strategy = typename access_mode::lock_strategy>
  class DirectAccess : public lock_strategy
  {
public:
    using MemoryBarrierType = MemoryBarrier<T>;

    explicit DirectAccess(MemoryBarrierType & mem_barrier) noexcept
    : lock_strategy(mem_barrier.mutex_), mem_(&mem_barrier), obj_(nullptr)
    {
      obj_ = _get(); // will return nullptr if we dont own the lock yet
    }

    template<class X>
    explicit DirectAccess(X & mem_barrier) noexcept
    : lock_strategy(mem_barrier.memory().mutex_), mem_(&mem_barrier.memory()), obj_(nullptr)
    {

      obj_ = _get(); // will return nullptr if we dont own the lock yet
    }

    template<class X>
    explicit DirectAccess(X * mem_barrier) noexcept
    : lock_strategy(mem_barrier->memory().mutex_), mem_(&mem_barrier->memory()), obj_(nullptr)
    {
      obj_ = _get(); // will return nullptr if we dont own the lock yet
    }


    // do not allow copying, only move semantics
    DirectAccess(const DirectAccess &) = delete;

    DirectAccess & operator=(const DirectAccess &) = delete;

    DirectAccess(DirectAccess && move) noexcept
    : mem_(move.mem_), obj_(move.obj_)
    {
      move.mem_ = nullptr;
    }

    DirectAccess & operator=(DirectAccess && move) noexcept
    {
      mem_ = move.mem_;
      move.mem_ = nullptr;
      obj_ = move.obj_;
      return *this;
    }

    inline void reset()
    {
      if (lock_strategy::owns_lock()) {
        lock_strategy::unlock();
      }
      mem_ = nullptr;
      obj_ = nullptr;
    }

    inline bool new_data_available() const
    {
      return mem_->new_data_available();
    }

    inline void new_data_available(bool avail)
    {
      if (!lock_strategy::owns_lock()) {
        throw std::runtime_error("can't modify an unlocked MemoryBarrier");
      }
      mem_->new_data_available_ = avail;
    }

    inline void swap()
    {
      if (!std::unique_lock<std::mutex>::owns_lock()) {
        throw std::runtime_error("can't swap an unlocked MemoryBarrier");
      }
      mem_->swap();
      obj_ = nullptr;   // must explicitly set to null so _get doesnt return cached value
      obj_ = _get();
    }

    inline explicit operator bool() const
    {return mem_ != nullptr && get();}

    inline bool operator==(std::nullptr_t) const
    {return mem_ == nullptr && get();}

    inline bool operator!=(std::nullptr_t) const
    {return mem_ != nullptr && get();}

    // smartptr semantics
    inline T * get()
    {return _get();}

    inline const T * get() const
    {return _get();}

    inline T * operator->()
    {return get();}

    inline const T * operator->() const
    {return get();}

    inline T & operator*()
    {return *get();}

    inline const T & operator*() const
    {return *get();}

private:
    MemoryBarrierType * mem_;
    mutable T * obj_;

    template<class realtimeness = access_mode>
    inline typename std::enable_if_t<std::is_trivially_assignable<realtime, realtimeness>::value,
      T *>
    _get()
    {
      return obj_ ?
             obj_ :
             lock_strategy::owns_lock() ?
             obj_ = mem_->rt_ :
               nullptr;
    }

    template<class realtimeness = access_mode>
    inline typename std::enable_if_t<std::is_trivially_assignable<non_realtime,
      realtimeness>::value, T *>
    _get()
    {
      return obj_ ?
             obj_ :
             lock_strategy::owns_lock() ?
             obj_ = mem_->nrt_ :
               nullptr;
    }

    template<class realtimeness = access_mode>
    inline typename std::enable_if_t<std::is_trivially_assignable<realtime, realtimeness>::value,
      const T *>
    _get() const
    {
      return obj_ ?
             obj_ :
             lock_strategy::owns_lock() ?
             obj_ = mem_->rt_ :
               nullptr;
    }

    template<class realtimeness = access_mode>
    inline typename std::enable_if_t<std::is_trivially_assignable<non_realtime,
      realtimeness>::value, const T *>
    _get() const
    {
      return obj_ ?
             obj_ :
             lock_strategy::owns_lock() ?
             obj_ = mem_->nrt_ :
               nullptr;
    }

  };


  MemoryBarrier()
  : nrt_(new T()), rt_(new T()), polarity_(false), new_data_available_(false)
  {
  }

  explicit MemoryBarrier(const T & data)
  : nrt_(new T(data)), rt_(new T(data)), polarity_(false), new_data_available_(false)
  {
  }

  explicit MemoryBarrier(MemoryBarrier<T> && move) noexcept
  : nrt_(move.nrt_), rt_(move.rt_), polarity_(move.polarity_), new_data_available_(
      move.new_data_available_)
  {
    move.nrt_ = nullptr;
    move.rt_ = nullptr;
    move.polarity_ = false;
    move.new_data_available_ = false;
  }

  /*MemoryBarrier(const MemoryBarrier<T, read_locking_strategy, write_locking_strategy>& copy)
  : in_(new T(*copy.in_)), out_(new T(*copy.out_)), polarity_(copy.polarity_), new_data_available_(copy.new_data_available_) {} }*/

  // allow moving but not copying
  MemoryBarrier(const MemoryBarrier<T> &) = delete;

  MemoryBarrier & operator=(const MemoryBarrier<T> &) = delete;


  // todo: perhaps we need a new_data on both nrt and rt side
  // true if new data is available, depends on if the memory is being used for Read from RT or Write to RT mode
  inline bool new_data_available() const
  {return new_data_available_;}

  void initialize(const T & value)
  {
    wait_lock guard(mutex_);
    if (guard.owns_lock()) {
      *nrt_ = *rt_ = value;
    } else {
      throw std::runtime_error("request to initialize realtime barrier failed trying to lock");
    }
  }

protected:
  T * nrt_;
  T * rt_;
  bool polarity_;  // flipped each time barrier rotates (swaps)
  bool new_data_available_;

  // Set as mutable so that readFromNonRT() can be performed on a const buffer
  mutable std::mutex mutex_;

  inline void swap()
  {
    // swap pointers
    T * tmp = nrt_;
    nrt_ = rt_;
    rt_ = tmp;
    polarity_ = !polarity_;
  }

  friend class DirectAccess<realtime>;

  friend class DirectAccess<non_realtime>;
};

///@brief Create a barrier for reading data from a realtime thread
/// ReadBarrier implements reading data from a realtime thread using a MemoryBarrier. For example,
/// this is used to transfer state data out of hardware interfaces. The default constructor will
// create a new memory barrier for use. There is an alternate constructor if you want to create
// your own MemoryBarrier explicitly.
template<class T, class locking_strategy = wait_lock>
class ReadBarrier
{
public:
  using MemoryBarrierType = MemoryBarrier<T>;
  //using DirectAccessType = typename T::DirectAccess<>;

  ReadBarrier() noexcept
  : mem_(new MemoryBarrierType()), owns_mem(true)
  {
  }

  explicit ReadBarrier(MemoryBarrierType & mem_barrier) noexcept
  : mem_(&mem_barrier), owns_mem(false)
  {
  }

  virtual ~ReadBarrier()
  {
    if (owns_mem) {
      delete mem_;
    }
  }

  //inline operator MemoryBarrier&() { return *mem_; }

  ///@brief Access the underlying MemoryBarrier object
  inline MemoryBarrierType & memory()
  {return *mem_;}

  ///@brief Get current value from non-realtime side without affecting the barrier.
  /// Copy the current data into dest. No swap is performed and the new_data_available flag is unaffected.
  bool current(T & dest)
  {
    typename MemoryBarrierType::template DirectAccess<non_realtime, locking_strategy> direct(*mem_);
    if (direct) {
      dest = *direct;
      return true;
    } else {
      return false;
    }
  }

  ///@brief Read data out of the realtime thread
  /// Swap RT buffer for non-RT. Copy the new data into val and reset the new_data_available flag.
  // todo: since this read also swaps, should it be called read_and_swap() or pop() or pull()
  bool pull(T & dest)
  {
    typename MemoryBarrierType::template DirectAccess<non_realtime, locking_strategy> direct(*mem_);
    if (direct && mem_->new_data_available()) {
      direct.swap();
      direct.new_data_available(false);
      dest = *direct;
      return true;
    } else {
      return false; // failed to read
    }
  }

private:
  MemoryBarrierType * mem_;
  bool owns_mem;
};


///@brief Create a barrier for writing data to a realtime thread
/// WriteBarrier implements writing data to a realtime thread from a non-realtime thread using a
/// MemoryBarrier. For example, this is used to transfer commands to a hardware interface. The
/// default constructor will create a new memory barrier for use. There is an alternate
/// constructor if you want to create your own MemoryBarrier explicitly.
template<class T, class locking_strategy = wait_lock>
class WriteBarrier
{
public:
  using MemoryBarrierType = MemoryBarrier<T>;
  //using DirectAccessType = DirectAccess<T>;

  WriteBarrier() noexcept
  : mem_(new MemoryBarrierType())
  {
  }

  WriteBarrier(MemoryBarrierType & mem_barrier) noexcept
  : mem_(&mem_barrier)
  {
  }

  ///@brief Access the underlying MemoryBarrier object
  inline MemoryBarrierType & memory()
  {return *mem_;}

  ///@brief Get current value from non-realtime side without affecting the barrier.
  /// Copy the current data into dest. No swap is performed and the new_data_available flag
  /// is unaffected.
  bool current(T & dest)
  {
    typename MemoryBarrierType::template DirectAccess<non_realtime, locking_strategy> direct(*mem_);
    if (direct) {
      dest = *direct;
      return true;
    } else {
      return false;
    }
  }

  ///@brief Write data into the realtime thread
  /// Copy the new data from val into non-RT buffer then swap RT buffer for non-RT and set the
  // new_data_available flag.
  bool push(const T & data)
  {
    typename MemoryBarrierType::template DirectAccess<non_realtime, locking_strategy> direct(*mem_);
    if (direct) {
      *direct = data;
      direct.swap();
      direct.new_data_available(true);
      return true;
    } else {
      return false;
    }
  }

private:
  MemoryBarrierType * mem_;
};

}


#endif // REALTIME_TOOLS__REALTIME_BARRIER_H

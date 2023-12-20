// concat of the source examples from the CLion C/C++ Code Style settings

// Tabs and Indents

#if !defined(OS)
#define OS_NOT_DEFINED
#endif
/*********************************************
 * ...globalFunc...
 *********************************************/
void globalFunc();
namespace foo {
/**
...Foo...
*/
class Foo {
 public:
  Foo();
  ~Foo();
  virtual Foo *getSelf() { return Foo::getSelf(); }

 private:
  void innerFunc();
  int var;
};
}  // namespace foo
struct FooPOD {
#ifdef OS_NOT_DEFINED
#define OS "unknown"
#endif
#define FooPOD_OS OS
  int i;
};
struct FooC {
 private:
  int i;
};
extern int a;
static int innerFunc();
int a = innerFunc();
int innerFunc() { return 5; }

void foo::Foo::innerFunc() {
label1:
  int continuation = 0xCD + 0xFD + 0xBAADF00D + 0xDEADBEEF;
  auto la = [](int i1, int i2) -> bool mutable {
    label2:
      return i1 < i2;
  }(1, 2);
}

// Spaces

#include <stdio.h>
#define min(a, b) ((a) < (b) ? (a) : (b))

template <typename T, typename M>
inline T const &Min(T const &a, M const &b) {
  return a < b ? a : b;
}

template <typename T>
class list {};
template <typename K, typename V = list<K>>
class hash {};
template <class T>
struct FooT {
  char g();
  hash<int, list<char>> elems;
  template <int N>
  int foo() {
    return N;
  }
  template <>
  int foo<2>() {
    return Min<>(1, 5);
  }
  list<int> mem = {1, 2, 3};
  float vector[3] = {};
  FooT() : elems{{-1, {'c', 'p', 'p'}}, {1, {'j', 'b'}}}, vector{1f, 2f, 3f} {}
  FooT operator++(int) volatile { return *this; }
  auto f(T t) -> decltype(t + g()) { return t + g(); }
};

class Bar {};
struct FooBase {};
int doSomething(...) { return 1; }

struct Foo : private FooBase {
 public:
  int i;
  Foo(const Foo &) = delete;
  Foo &operator=(const Foo &) = default;
  void log() volatile &;
  virtual int action(int, char, float) = 0;
  virtual Foo *getSelf() const { return this; }

 private:
  static int method(){};
};

namespace fooNS {
class FooClass : Foo, virtual FooBase {
  typedef int (FooClass::*ACTION)(int);

 public:
  FooClass() { act = &FooClass::nv_action; }
  virtual ~FooClass(){};
  int nv_action(int arg) { return arg; }
  virtual int action(int color, char alpha, float);
  virtual Foo *getSelf() { return Foo::getSelf(); }
  int method() {
    FooT<int> bar;
    int X[] = {1, 3, 5, 6, 7, 87, 1213, 2};
    int W[][3] = {{1, 3, 5}, {6, 7, 8}};
    int y = 0, x;
    auto la = [X, W](int i1, int i2) -> bool mutable { return i1 < i2; }(1, 2);
    auto laF = []() {};
    auto &[bb, cc, dd] = W[0];
    bool z = (bar.foo<2>() & 4) == 4;
    for (int z : {1, 2, 3}) {
    }
    for (int i = 0; i < x; i++) {
      y += (y ^ 0x123) << 2;
    }
    do {
      try {
        if (0 < x && x < 10) {
          while (x != y) {
            x = min(x * 3, 5);
          }
          x = x >= 5 ?: 5;
        } else if (min(1, 5) == 1) {
          switch (this->action(0xFeeL, 0120, 0.01F)) {
            default:
              break;
            case 1:
              continue;
          }
        }
      } catch (char *message) {
        const int *arr = X;
        x = ((y >= 0) ? arr[y] : -1);
      }
    } while (true);
  }
  ACTION act;

 private:
  int var;
};
}  // namespace fooNS

int fooNS::FooClass::action(int color, char alpha, float) {
  fooNS::FooClass object,
      *ptr = (fooNS::FooClass *)object.getSelf()->getSelf()->getSelf();
  (object.*object.act)(1);
  ptr->action(10001, '\x1A', 1e13f);
  ptr->getSelf()->getSelf()->getSelf()->getSelf();
  return doSomething(color);
}

typedef void(fn)(int i, int j, int k);
typedef void (*block)(int i, int j, int k);
typedef int X;
int &refTest(X &&x, int y, int b, void *(*)()) {
  int **&p = (int **&)x;
  int static &r = *&x;
  return r && (r & x) ? r : x;
}

struct fooS {
  int i : 4;
  char j;
} foo_t;
enum fooE {
  SUNDAY = 111,
  MONDAY = 222,
  TUESDAY = 333,
  WEDNESDAY = TUESDAY + 1
} foo_e;

struct fooOp {
  template <typename T>
  fooOp &operator&(const T &i) {
    std::cout << std::forward<T>(t);
    return *this;
  }
  void dump() { *this & 1 & 2 & 3 & "4" & 5.0f & i_ &d_ &f_ &s_; }
  int i_;
  double d_;
  float f_;
  std::string s_;
};
void useFooOp() {
  fooOp x;
  int i;
  x & 1 & 2 & 3 & "4" & 5.0f & i;
}

// Wrapping and Braces

#include <stdio.h>
#define min(a, b) ((a) < (b) ? (a) : (b))

template <class T>
class list {};

class Bar {};
struct FooBase {};
int doSomething(...);
int doSomethingElse(...) { return 2; }

struct Foo : private FooBase {
 public:
  int i;
  virtual int action(int, char, float) = 0;
  virtual Foo *getSelf() { return this; }

 private:
  static int method(){};
  list<Bar> bar;
};

namespace fooNS {
class FooClass : Foo, virtual FooBase {
// comment start ar first column
#define FooClass_B FooBase
  typedef int (FooClass::*ACTION)(int);

 public:
  FooClass() { act = &FooClass::nv_action; }
  virtual ~FooClass() {}
  int nv_action(int arg) { return arg; }
  virtual int action(int color, char alpha, float);
  virtual Foo *getSelf() { return Foo::getSelf(); }
  int method() { return 0; }
  ACTION act;

 private:
  int var;
};
}  // namespace fooNS

int fooNS::FooClass::action(int color, char alpha, float) {
  return doSomething(color);
}

namespace A {
namespace B {
typedef void(fn)(int i, int j, int k);
typedef void (*block)(int i, int j, int k);
}  // namespace B
}  // namespace A
typedef int X;
int &refTest(X &&x) {
  int **&p = (int **&)x;
  int static &r = *&x;
  return r && (r & x) ? r : x;
}

// todo something
void doSomething(int y, int b, void *(*)()) {
  int a = 1 || 0 && 1;
  int bb = a == !1 && a != 0;
  int ccc = bb = a = 1 < 2 >= 3;
  int dddd = ccc = bb = a = ~1 | 2 & 3 ^ 4;

  void *p1 = reinterpret_cast<void *>(&a);
  void **p2 = &p1;

  a = bb = ccc = dddd = 2;
  dddd = ccc = bb = a = (1 + 2 + 3 + 4 + 5 + 0xFFFFFFFFF);

  int i5 = ((1) + 2) - (4 * 5 / 6 % 7);
  int i6 = -123456 << 2 >> 3 >> 12345;
  int i7 = 2 > 3 ? 7 + 8 + 9 : 11 + 12 + 13;
  int i8 = 2 < 3 + 7 + 8 + 9 ?: 11 + 12 + 13;
  int ii[6], jj[] = {1, 2, 3, 0x000A, 0x000B, 0x000C};

  fooNS::FooClass object,
      *ptr = (fooNS::FooClass *)object.getSelf()->getSelf()->getSelf();
  (object.*object.act)(1);
  ptr->action(0xFF0000, 0.01, 320);
  ptr->getSelf()->getSelf()->getSelf()->getSelf();

  doSomething(ii[1], jj[ii[2]], doSomething(123));

  if (1)
    doSomething(1);
  else if (2)
    doSomething(1, 2);
  if (1) {
    doSomething();
  } else if (2) {
    doSomething();
  } else
    doSomething();
  for (int i = 1, j = 2; i <= j; i++, j--) doSomethingElse();
  while (1) doSomethingElse();
  do doSomethingElse();
  while (1);
  switch (1) {
    case 0:
      return;
    case 1: {
      return;
    }
  }

  try {
    doSomethingElse();
  } catch (char *message) {
    return;
  }
}

struct fooS {
  int i : 4;
  char j;
} foo_t;
enum fooE {
  SUNDAY = 111,
  MONDAY = 222,
  TUESDAY = 333,
  WEDNESDAY = TUESDAY + 1
} foo_e;

template <typename T, typename M>
inline T const &Min(T const &a, M const &b) {
  return a < b ? a : b;
}

template <typename K, typename V = list<K>>
class hash {};
template <class T>
struct FooT {
  hash<int, list<char>> elems;
  template <int N>
  int foo() {
    return N;
  }
  template <>
  int foo<2>() {
    return Min<>(1, 5);
  }
  list<int> mem = {1, 2, 3};
  float vector[3];
  FooT() : elems{{-1, {'c', 'p', 'p'}}, {1, {'j', 'b'}}}, vector{1f, 2f, 3f} {
    auto la = [=, this, &mem, elems](int i1, int i2) -> bool mutable {
      return i1 < i2;
    }(1, 2);
    auto &[i, c, d] = vector;
  }
  auto f(T t) -> decltype(t + doSomething()) { return t + doSomething(); }
};

// Blank Lines

#include <vector>
class Foo {
  friend class AnotherClass;

 private:
  Foo *field1;

 public:
  int field2;
  Foo() { field1 = new Foo(); }
  class InnerClass {
   public:
    static int x;
    static int y;
    int f();
    int g();
  };
};
int Foo::InnerClass::x = 25;
int Foo::InnerClass::f() { return 0; };
typedef Foo::InnerClass owner;  // define a typedef
int owner::y = 11;              // use typedef with ::
int owner::g() { return 0; };
class AnotherClass {};
struct TestInterface {
  static const int MAX = 42;
  static const int MIN = 0;
  virtual void method1() = 0;
  virtual void method2() = 0;
};

///////////////// REAL WORLD EXAMPLE 1: TTG's make_tt.h

// to be #include'd within runtime::ttg namespace

#ifndef TTG_MAKE_TT_H
#define TTG_MAKE_TT_H

// Class to wrap a callable with signature
//
// case 1 (keyT != void): void op(auto&& key, std::tuple<input_valuesT...>&&,
// std::tuple<output_terminalsT...>&) case 2 (keyT == void): void
// op(std::tuple<input_valuesT...>&&, std::tuple<output_terminalsT...>&)
//
template <typename funcT, bool funcT_receives_outterm_tuple, typename keyT,
          typename output_terminalsT, typename... input_valuesT>
class CallableWrapTT
    : public TT<keyT, output_terminalsT,
                CallableWrapTT<funcT, funcT_receives_outterm_tuple, keyT,
                               output_terminalsT, input_valuesT...>,
                ttg::typelist<input_valuesT...>> {
  using baseT = typename CallableWrapTT::ttT;

  using input_values_tuple_type = typename baseT::input_values_tuple_type;
  using input_refs_tuple_type = typename baseT::input_refs_tuple_type;
  using input_edges_type = typename baseT::input_edges_type;
  using output_edges_type = typename baseT::output_edges_type;

  using noref_funcT = std::remove_reference_t<funcT>;
  std::conditional_t<std::is_function_v<noref_funcT>,
                     std::add_pointer_t<noref_funcT>, noref_funcT>
      func;

  template <typename Key, typename Tuple>
  void call_func(Key &&key, Tuple &&args, output_terminalsT &out) {
    if constexpr (funcT_receives_outterm_tuple)
      func(std::forward<Key>(key), std::forward<Tuple>(args), out);
    else {
      auto old_output_tls_ptr = this->outputs_tls_ptr_accessor();
      this->set_outputs_tls_ptr();
      func(std::forward<Key>(key), std::forward<Tuple>(args));
      this->set_outputs_tls_ptr(old_output_tls_ptr);
    }
  }

  template <typename TupleOrKey>
  void call_func(TupleOrKey &&args, output_terminalsT &out) {
    if constexpr (funcT_receives_outterm_tuple)
      func(std::forward<TupleOrKey>(args), out);
    else {
      auto old_output_tls_ptr = this->outputs_tls_ptr_accessor();
      this->set_outputs_tls_ptr();
      func(std::forward<TupleOrKey>(args));
      this->set_outputs_tls_ptr(old_output_tls_ptr);
    }
  }

  void call_func(output_terminalsT &out) {
    if constexpr (funcT_receives_outterm_tuple)
      func(std::tuple<>(), out);
    else {
      auto old_output_tls_ptr = this->outputs_tls_ptr_accessor();
      this->set_outputs_tls_ptr();
      func(std::tuple<>());
      this->set_outputs_tls_ptr(old_output_tls_ptr);
    }
  }

 public:
  template <typename funcT_>
  CallableWrapTT(funcT_ &&f, const input_edges_type &inedges,
                 const output_edges_type &outedges, const std::string &name,
                 const std::vector<std::string> &innames,
                 const std::vector<std::string> &outnames)
      : baseT(inedges, outedges, name, innames, outnames),
        func(std::forward<funcT_>(f)) {}

  template <typename funcT_>
  CallableWrapTT(funcT_ &&f, const std::string &name,
                 const std::vector<std::string> &innames,
                 const std::vector<std::string> &outnames)
      : baseT(name, innames, outnames), func(std::forward<funcT_>(f)) {}

  template <typename Key, typename ArgsTuple>
  std::enable_if_t<std::is_same_v<ArgsTuple, input_refs_tuple_type> &&
                       !ttg::meta::is_empty_tuple_v<ArgsTuple> &&
                       !ttg::meta::is_void_v<Key>,
                   void>
  op(Key &&key, ArgsTuple &&args_tuple, output_terminalsT &out) {
    call_func(std::forward<Key>(key), std::forward<ArgsTuple>(args_tuple), out);
  }

  template <typename ArgsTuple, typename Key = keyT>
  std::enable_if_t<std::is_same_v<ArgsTuple, input_refs_tuple_type> &&
                       !ttg::meta::is_empty_tuple_v<ArgsTuple> &&
                       ttg::meta::is_void_v<Key>,
                   void>
  op(ArgsTuple &&args_tuple, output_terminalsT &out) {
    call_func(std::forward<ArgsTuple>(args_tuple), out);
  }

  template <typename Key, typename ArgsTuple = input_values_tuple_type>
  std::enable_if_t<ttg::meta::is_empty_tuple_v<ArgsTuple> &&
                       !ttg::meta::is_void_v<Key>,
                   void>
  op(Key &&key, output_terminalsT &out) {
    call_func(std::forward<Key>(key), out);
  }

  template <typename Key = keyT, typename ArgsTuple = input_values_tuple_type>
  std::enable_if_t<
      ttg::meta::is_empty_tuple_v<ArgsTuple> && ttg::meta::is_void_v<Key>, void>
  op(output_terminalsT &out) {
    call_func(out);
  }
};

template <typename funcT, bool funcT_receives_outterm_tuple, typename keyT,
          typename output_terminalsT, typename input_values_tupleT>
struct CallableWrapTTUnwrapTypelist;

template <typename funcT, bool funcT_receives_outterm_tuple, typename keyT,
          typename output_terminalsT, typename... input_valuesT>
struct CallableWrapTTUnwrapTypelist<funcT, funcT_receives_outterm_tuple, keyT,
                                    output_terminalsT,
                                    std::tuple<input_valuesT...>> {
  using type = CallableWrapTT<funcT, funcT_receives_outterm_tuple, keyT,
                              output_terminalsT,
                              std::remove_reference_t<input_valuesT>...>;
};

template <typename funcT, bool funcT_receives_outterm_tuple, typename keyT,
          typename output_terminalsT, typename... input_valuesT>
struct CallableWrapTTUnwrapTypelist<funcT, funcT_receives_outterm_tuple, keyT,
                                    output_terminalsT,
                                    ttg::meta::typelist<input_valuesT...>> {
  using type = CallableWrapTT<funcT, funcT_receives_outterm_tuple, keyT,
                              output_terminalsT,
                              std::remove_reference_t<input_valuesT>...>;
};

// Class to wrap a callable with signature
//
// case 1 (keyT != void): returnT op(auto&& key, input_valuesT&&...,
// std::tuple<output_terminalsT...>&) case 2 (keyT == void): returnT
// op(input_valuesT&&..., std::tuple<output_terminalsT...>&)
//
// returnT is void for funcT = synchronous (ordinary) function and the
// appropriate return type for funcT=coroutine
template <typename funcT, typename returnT, bool funcT_receives_outterm_tuple,
          ttg::ExecutionSpace space, typename keyT, typename output_terminalsT,
          typename... input_valuesT>
class CallableWrapTTArgs
    : public TT<
          keyT, output_terminalsT,
          CallableWrapTTArgs<funcT, returnT, funcT_receives_outterm_tuple,
                             space, keyT, output_terminalsT, input_valuesT...>,
          ttg::typelist<input_valuesT...>> {
  using baseT = typename CallableWrapTTArgs::ttT;

  using input_values_tuple_type = typename baseT::input_values_tuple_type;
  using input_refs_tuple_type = typename baseT::input_refs_tuple_type;
  using input_edges_type = typename baseT::input_edges_type;
  using output_edges_type = typename baseT::output_edges_type;

  using noref_funcT = std::remove_reference_t<funcT>;
  std::conditional_t<std::is_function_v<noref_funcT>,
                     std::add_pointer_t<noref_funcT>, noref_funcT>
      func;

  using op_return_type =
#ifdef TTG_HAS_COROUTINE
      std::conditional_t<
          std::is_same_v<returnT, ttg::resumable_task>, ttg::coroutine_handle<>,
#ifdef TTG_HAVE_DEVICE
          std::conditional_t<std::is_same_v<returnT, ttg::device_task>,
                             ttg::device_task::base_type, void>
#else   // TTG_HAVE_DEVICE
          void
#endif  // TTG_HAVE_DEVICE
          >;
#else   // TTG_HAS_COROUTINE
      void;
#endif  // TTG_HAS_COROUTINE

 public:
  static constexpr bool have_cuda_op = (space == ttg::ExecutionSpace::CUDA);
  static constexpr bool have_hip_op = (space == ttg::ExecutionSpace::HIP);
  static constexpr bool have_level_zero_op = (space == ttg::ExecutionSpace::L0);

 protected:
  template <typename ReturnT>
  auto process_return(ReturnT &&ret, output_terminalsT &out) {
    static_assert(
        std::is_same_v<std::remove_reference_t<decltype(ret)>, returnT>,
        "CallableWrapTTArgs<funcT,returnT,...>: returnT does not match the "
        "actual return type of funcT");
    if constexpr (!std::is_void_v<returnT>) {  // protect from compiling for
                                               // void returnT
#ifdef TTG_HAS_COROUTINE
      if constexpr (std::is_same_v<returnT, ttg::resumable_task>) {
        ttg::coroutine_handle<> coro_handle;
        // if task completed destroy it
        if (ret.completed()) {
          ret.destroy();
        } else {  // if task is suspended return the coroutine promise ptr
          coro_handle = ret;
        }
        return coro_handle;
      } else
#ifdef TTG_HAVE_DEVICE
          if constexpr (std::is_same_v<returnT, ttg::device_task>) {
        ttg::device_task::base_type coro_handle = ret;
        return coro_handle;
      }
#else   // TTG_HAVE_DEVICE
        ttg::abort();  // should not happen
#endif  // TTG_HAVE_DEVICE
      if constexpr (!(std::is_same_v<returnT, ttg::resumable_task>
#ifdef TTG_HAVE_DEVICE
                      || std::is_same_v<returnT, ttg::device_task>
#endif  // TTG_HAVE_DEVICE
                      ))
#endif
      {
        static_assert(
            std::tuple_size_v<std::remove_reference_t<decltype(out)>> == 1,
            "CallableWrapTTArgs<funcT,returnT,funcT_receives_outterm_tuple="
            "true,...): funcT can return a "
            "value only if there is only 1 out terminal");
        static_assert(std::tuple_size_v<returnT> <= 2,
                      "CallableWrapTTArgs<funcT,returnT,funcT_receives_outterm_"
                      "tuple=true,...): funcT can return a "
                      "value only if it is a plain value (then sent with null "
                      "key), a tuple-like containing a single "
                      "key (hence value is void), or a tuple-like containing a "
                      "key and a value");
        if constexpr (std::tuple_size_v<returnT> == 0)
          std::get<0>(out).sendv(std::move(ret));
        else if constexpr (std::tuple_size_v<returnT> == 1)
          std::get<0>(out).sendk(std::move(std::get<0>(ret)));
        else if constexpr (std::tuple_size_v<returnT> == 2)
          std::get<0>(out).send(std::move(std::get<0>(ret)),
                                std::move(std::get<1>(ret)));
        return;
      }
    }
  }

  /// @return coroutine handle<> (if funcT is a coroutine), else void
  template <typename Key, typename Tuple, std::size_t... S>
  auto call_func(Key &&key, Tuple &&args_tuple, output_terminalsT &out,
                 std::index_sequence<S...>) {
    using func_args_t =
        ttg::meta::tuple_concat_t<std::tuple<const Key &>,
                                  input_refs_tuple_type, output_edges_type>;

    if constexpr (funcT_receives_outterm_tuple) {
      if constexpr (std::is_void_v<returnT>) {
        func(std::forward<Key>(key),
             baseT::template get<S, std::tuple_element_t<S + 1, func_args_t>>(
                 std::forward<Tuple>(args_tuple))...,
             out);
        return;
      } else {
        auto ret = func(
            std::forward<Key>(key),
            baseT::template get<S, std::tuple_element_t<S + 1, func_args_t>>(
                std::forward<Tuple>(args_tuple))...,
            out);

        return process_return(std::move(ret), out);
      }
    } else {
      auto old_output_tls_ptr = this->outputs_tls_ptr_accessor();
      this->set_outputs_tls_ptr();
      if constexpr (std::is_void_v<returnT>) {
        func(std::forward<Key>(key),
             baseT::template get<S, std::tuple_element_t<S + 1, func_args_t>>(
                 std::forward<Tuple>(args_tuple))...);
        this->set_outputs_tls_ptr(old_output_tls_ptr);
        return;
      } else {
        auto ret = func(
            std::forward<Key>(key),
            baseT::template get<S, std::tuple_element_t<S + 1, func_args_t>>(
                std::forward<Tuple>(args_tuple))...);
        this->set_outputs_tls_ptr(old_output_tls_ptr);
        return process_return(std::move(ret), out);
      }
    }
  }

  template <typename Tuple, std::size_t... S>
  auto call_func(Tuple &&args_tuple, output_terminalsT &out,
                 std::index_sequence<S...>) {
    using func_args_t =
        ttg::meta::tuple_concat_t<input_refs_tuple_type, output_edges_type>;
    if constexpr (funcT_receives_outterm_tuple) {
      if constexpr (std::is_void_v<returnT>) {
        func(baseT::template get<S, std::tuple_element_t<S, func_args_t>>(
                 std::forward<Tuple>(args_tuple))...,
             out);
      } else {
        auto ret =
            func(baseT::template get<S, std::tuple_element_t<S, func_args_t>>(
                     std::forward<Tuple>(args_tuple))...,
                 out);
        return process_return(std::move(ret), out);
      }
    } else {
      auto old_output_tls_ptr = this->outputs_tls_ptr_accessor();
      this->set_outputs_tls_ptr();
      if constexpr (std::is_void_v<returnT>) {
        func(baseT::template get<S, std::tuple_element_t<S, func_args_t>>(
            std::forward<Tuple>(args_tuple))...);
        this->set_outputs_tls_ptr(old_output_tls_ptr);
      } else {
        auto ret =
            func(baseT::template get<S, std::tuple_element_t<S, func_args_t>>(
                std::forward<Tuple>(args_tuple))...);
        this->set_outputs_tls_ptr(old_output_tls_ptr);
        return process_return(std::move(ret), out);
      }
    }
  }

  template <typename Key>
  auto call_func(Key &&key, output_terminalsT &out) {
    if constexpr (funcT_receives_outterm_tuple) {
      if constexpr (std::is_void_v<returnT>) {
        func(std::forward<Key>(key), out);
      } else {
        auto ret = func(std::forward<Key>(key), out);
        return process_return(std::move(ret), out);
      }
    } else {
      auto old_output_tls_ptr = this->outputs_tls_ptr_accessor();
      this->set_outputs_tls_ptr();
      if constexpr (std::is_void_v<returnT>) {
        func(std::forward<Key>(key));
        this->set_outputs_tls_ptr(old_output_tls_ptr);
      } else {
        auto ret = func(std::forward<Key>(key));
        this->set_outputs_tls_ptr(old_output_tls_ptr);
        return process_return(std::move(ret), out);
      }
    }
  }

  template <typename OutputTerminals>
  auto call_func(OutputTerminals &out) {
    if constexpr (funcT_receives_outterm_tuple) {
      if constexpr (std::is_void_v<returnT>) {
        func(out);
      } else {
        auto ret = func(out);
        return process_return(std::move(ret), out);
      }
    } else {
      auto old_output_tls_ptr = this->outputs_tls_ptr_accessor();
      this->set_outputs_tls_ptr();
      if constexpr (std::is_void_v<returnT>) {
        func();
        this->set_outputs_tls_ptr(old_output_tls_ptr);
      } else {
        auto ret = func(out);
        this->set_outputs_tls_ptr(old_output_tls_ptr);
        return process_return(std::move(ret), out);
      }
    }
  }

  template <typename Tuple, std::size_t... I>
  static auto make_output_terminal_ptrs(const Tuple &output_terminals,
                                        std::index_sequence<I...>) {
    return std::array<ttg::TerminalBase *, sizeof...(I)>{
        {static_cast<ttg::TerminalBase *>(&std::get<I>(output_terminals))...}};
  }

 public:
  template <typename funcT_>
  CallableWrapTTArgs(funcT_ &&f, const input_edges_type &inedges,
                     const typename baseT::output_edges_type &outedges,
                     const std::string &name,
                     const std::vector<std::string> &innames,
                     const std::vector<std::string> &outnames)
      : baseT(inedges, outedges, name, innames, outnames),
        func(std::forward<funcT_>(f)) {}

  template <typename funcT_>
  CallableWrapTTArgs(funcT_ &&f, const std::string &name,
                     const std::vector<std::string> &innames,
                     const std::vector<std::string> &outnames)
      : baseT(name, innames, outnames), func(std::forward<funcT_>(f)) {}

  template <typename Key, typename ArgsTuple>
  std::enable_if_t<std::is_same_v<ArgsTuple, input_refs_tuple_type> &&
                       !ttg::meta::is_empty_tuple_v<input_refs_tuple_type> &&
                       !ttg::meta::is_void_v<Key>,
                   op_return_type>
  op(Key &&key, ArgsTuple &&args_tuple, output_terminalsT &out) {
    assert(&out == &baseT::get_output_terminals());
    return call_func(std::forward<Key>(key),
                     std::forward<ArgsTuple>(args_tuple), out,
                     std::make_index_sequence<std::tuple_size_v<ArgsTuple>>{});
  };

  template <typename ArgsTuple, typename Key = keyT>
  std::enable_if_t<std::is_same_v<ArgsTuple, input_refs_tuple_type> &&
                       !ttg::meta::is_empty_tuple_v<input_refs_tuple_type> &&
                       ttg::meta::is_void_v<Key>,
                   op_return_type>
  op(ArgsTuple &&args_tuple, output_terminalsT &out) {
    assert(&out == &baseT::get_output_terminals());
    return call_func(std::forward<ArgsTuple>(args_tuple), out,
                     std::make_index_sequence<std::tuple_size_v<ArgsTuple>>{});
  };

  template <typename Key, typename ArgsTuple = input_refs_tuple_type>
  std::enable_if_t<ttg::meta::is_empty_tuple_v<ArgsTuple> &&
                       !ttg::meta::is_void_v<Key>,
                   op_return_type>
  op(Key &&key, output_terminalsT &out) {
    assert(&out == &baseT::get_output_terminals());
    return call_func(std::forward<Key>(key), out);
  };

  template <typename Key = keyT, typename ArgsTuple = input_refs_tuple_type>
  std::enable_if_t<ttg::meta::is_empty_tuple_v<ArgsTuple> &&
                       ttg::meta::is_void_v<Key>,
                   op_return_type>
  op(output_terminalsT &out) {
    assert(&out == &baseT::get_output_terminals());
    return call_func(out);
  };
};

template <typename funcT, typename returnT, bool funcT_receives_outterm_tuple,
          ttg::ExecutionSpace space, typename keyT, typename output_terminalsT,
          typename input_values_typelistT>
struct CallableWrapTTArgsAsTypelist;

template <typename funcT, typename returnT, bool funcT_receives_outterm_tuple,
          ttg::ExecutionSpace space, typename keyT, typename output_terminalsT,
          typename... input_valuesT>
struct CallableWrapTTArgsAsTypelist<
    funcT, returnT, funcT_receives_outterm_tuple, space, keyT,
    output_terminalsT, std::tuple<input_valuesT...>> {
  using type = CallableWrapTTArgs<funcT, returnT, funcT_receives_outterm_tuple,
                                  space, keyT, output_terminalsT,
                                  std::remove_reference_t<input_valuesT>...>;
};

template <typename funcT, typename returnT, bool funcT_receives_outterm_tuple,
          ttg::ExecutionSpace space, typename keyT, typename output_terminalsT,
          typename... input_valuesT>
struct CallableWrapTTArgsAsTypelist<
    funcT, returnT, funcT_receives_outterm_tuple, space, keyT,
    output_terminalsT, ttg::meta::typelist<input_valuesT...>> {
  using type = CallableWrapTTArgs<funcT, returnT, funcT_receives_outterm_tuple,
                                  space, keyT, output_terminalsT,
                                  std::remove_reference_t<input_valuesT>...>;
};

// clang-format off
/// @brief Factory function to assist in wrapping a callable with signature
///
/// @tparam keyT a task ID type
/// @tparam funcT a callable type
/// @tparam input_edge_valuesT a pack of types of input data
/// @tparam output_edgesT a pack of types of output edges
/// @param[in] func a callable object; it can be _generic_ (e.g., a template function, a generic lambda, etc.; see
///            below) or _nongeneric_ (with concrete types for its arguments). In either case its signature
///            must match the following:
///         - if `ttg::meta::is_void_v<keyT>==true`:
///           - `void(const std::tuple<input_valuesT&...>&, std::tuple<output_terminalsT...>&)`: full form, with the explicitly-passed
///             output terminals ensuring compile-time type-checking of the dataflow into the output terminals (see ttg::send);
///           - `void(const std::tuple<input_valuesT&...>&)`: simplified form, with no type-checking of the dataflow into the output terminals;
///         - if `ttg::meta::is_void_v<keyT>==false`:
///           - `void(const keyT&, const std::tuple<input_valuesT&...>&, std::tuple<output_terminalsT...>&)`: full form, with the explicitly-passed
/////             output terminals ensuring compile-time type-checking of the dataflow into the output terminals (see ttg::send);
///           - `void(const keyT&, const std::tuple<input_valuesT&...>&)`: simplified form, with no type-checking of the dataflow into the output terminals.
/// @param[in] inedges a tuple of input edges
/// @param[in] outedges a tuple of output edges
/// @param[in] name a string label for the resulting TT
/// @param[in] name a string label for the resulting TT
/// @param[in] innames string labels for the respective input terminals of the resulting TT
/// @param[in] outnames string labels for the respective output terminals of the resulting TT
///
/// @note Handling of generic @p func is described in the documentation of make_tt()
// clang-format on
template <typename keyT = void, typename funcT, typename... input_edge_valuesT,
          typename... output_edgesT>
auto make_tt_tpl(
    funcT &&func,
    const std::tuple<ttg::Edge<keyT, input_edge_valuesT>...> &inedges =
        std::tuple<>{},
    const std::tuple<output_edgesT...> &outedges = std::tuple<>{},
    const std::string &name = "wrapper",
    const std::vector<std::string> &innames =
        std::vector<std::string>(sizeof...(input_edge_valuesT), "input"),
    const std::vector<std::string> &outnames =
        std::vector<std::string>(sizeof...(output_edgesT), "output")) {
  // ensure input types do not contain Void
  static_assert(ttg::meta::is_none_Void_v<input_edge_valuesT...>,
                "ttg::Void is for internal use only, do not use it");
  using output_terminals_type = typename ttg::edges_to_output_terminals<
      std::tuple<output_edgesT...>>::type;

  constexpr auto void_key = ttg::meta::is_void_v<keyT>;

  // list of base datum types (T or const T)
  using base_input_data_t = ttg::meta::typelist<
      typename ttg::Edge<keyT, input_edge_valuesT>::value_type...>;

  // gross list of candidate argument types
  using gross_candidate_func_args_t = ttg::meta::typelist<
      ttg::meta::candidate_argument_bindings_t<std::add_const_t<keyT>>,
      ttg::meta::candidate_argument_bindings_t<
          std::tuple<std::add_lvalue_reference_t<
              typename ttg::Edge<keyT, input_edge_valuesT>::value_type>...>>,
      ttg::meta::typelist<output_terminals_type &, void>>;

  // net list of candidate argument types excludes the empty typelists for void
  // arguments
  using candidate_func_args_t =
      ttg::meta::filter_t<gross_candidate_func_args_t,
                          ttg::meta::typelist_is_not_empty>;

  // compute list of argument types with which func can be invoked
  constexpr static auto func_is_generic =
      ttg::meta::is_generic_callable_v<funcT>;
  using gross_func_args_t =
      decltype(ttg::meta::compute_arg_binding_types_r<void>(
          func, candidate_func_args_t{}));
  constexpr auto DETECTED_HOW_TO_INVOKE_GENERIC_FUNC =
      func_is_generic ? !std::is_same_v<gross_func_args_t, ttg::typelist<>>
                      : true;
  static_assert(
      DETECTED_HOW_TO_INVOKE_GENERIC_FUNC,
      "ttd::make_tt_tpl(func, inedges, ...): could not detect how to invoke "
      "generic callable func, either "
      "the signature of func "
      "is faulty, or inedges does match the expected list of types, or both");

  // net argument typelist
  using func_args_t = ttg::meta::drop_void_t<gross_func_args_t>;
  constexpr auto num_args = std::tuple_size_v<func_args_t>;

  // if given task id, make sure it's passed via const lvalue ref
  constexpr bool TASK_ID_PASSED_AS_CONST_LVALUE_REF =
      !void_key ? ttg::meta::probe_first_v<ttg::meta::is_const_lvalue_reference,
                                           true, func_args_t>
                : true;
  constexpr bool TASK_ID_PASSED_AS_NONREF =
      !void_key
          ? !ttg::meta::probe_first_v<std::is_reference, true, func_args_t>
          : true;
  static_assert(TASK_ID_PASSED_AS_CONST_LVALUE_REF || TASK_ID_PASSED_AS_NONREF,
                "ttg::make_tt_tpl(func, ...): if given to func, the task id "
                "must be passed by const lvalue ref or by value");

  // if given out-terminal tuple, make sure it's passed via nonconst lvalue ref
  constexpr bool have_outterm_tuple =
      func_is_generic
          ? !ttg::meta::is_last_void_v<gross_func_args_t>
          : ttg::meta::probe_last_v<
                ttg::meta::
                    is_nonconst_lvalue_reference_to_output_terminal_tuple,
                true, gross_func_args_t>;
  constexpr bool OUTTERM_TUPLE_PASSED_AS_NONCONST_LVALUE_REF =
      have_outterm_tuple
          ? ttg::meta::probe_last_v<ttg::meta::is_nonconst_lvalue_reference,
                                    true, func_args_t>
          : true;
  static_assert(OUTTERM_TUPLE_PASSED_AS_NONCONST_LVALUE_REF,
                "ttd::make_tt_tpl(func, ...): if given to func, the output "
                "terminal tuple must be passed by nonconst lvalue ref");

  static_assert(
      num_args == 3 - (void_key ? 1 : 0) - (have_outterm_tuple ? 0 : 1),
      "ttg::make_tt_tpl(func, ...): func takes wrong number of arguments (2, "
      "or 1, if keyT=void + optional "
      "tuple of output terminals)");

  // 2. input_args_t = {input_valuesT&&...}
  using nondecayed_input_args_t =
      std::tuple_element_t<void_key ? 0 : 1, func_args_t>;
  constexpr auto NO_ARGUMENTS_PASSED_AS_NONCONST_LVALUE_REF =
      !ttg::meta::is_any_nonconst_lvalue_reference_v<nondecayed_input_args_t>;
  static_assert(NO_ARGUMENTS_PASSED_AS_NONCONST_LVALUE_REF,
                "ttg::make_tt_tpl(func, inedges, outedges): one or more "
                "arguments to func can only be passed by nonconst lvalue "
                "ref; this is illegal, should only pass arguments as const "
                "lvalue ref or (nonconst) rvalue ref");
  using input_args_t = std::decay_t<nondecayed_input_args_t>;
  using decayed_input_args_t = ttg::meta::decayed_typelist_t<input_args_t>;
  using wrapT =
      typename CallableWrapTTUnwrapTypelist<funcT, have_outterm_tuple, keyT,
                                            output_terminals_type,
                                            input_args_t>::type;
  static_assert(
      std::is_same_v<decayed_input_args_t, std::tuple<input_edge_valuesT...>>,
      "ttg::make_tt_tpl(func, inedges, outedges): inedges value types do not "
      "match argument types of func");

  return std::make_unique<wrapT>(std::forward<funcT>(func), inedges, outedges,
                                 name, innames, outnames);
}

// clang-format off
/// @brief Factory function to assist in wrapping a callable with signature
///
/// @tparam keyT a task ID type
/// @tparam funcT a callable type
/// @tparam input_edge_valuesT a pack of types of input data
/// @tparam output_edgesT a pack of types of output edges
/// @param[in] func a callable object; it can be _generic_ (e.g., a template function, a generic lambda, etc.; see
///            below) or _nongeneric_ (with concrete types for its arguments). In either case its signature
///            must match the following:
///         - if `ttg::meta::is_void_v<keyT>==true`:
///           - `void(input_valuesT&&..., std::tuple<output_terminalsT...>&)`: full form, with the explicitly-passed
///             output terminals ensuring compile-time type-checking of the dataflow into the output terminals (see ttg::send);
///           - `void(input_valuesT&&...)`: simplified form, with no type-checking of the dataflow into the output terminals;
///         - if `ttg::meta::is_void_v<keyT>==false`:
///           - `void(const keyT&, input_valuesT&&..., std::tuple<output_terminalsT...>&)`: full form, with the explicitly-passed
///             output terminals ensuring compile-time type-checking of the dataflow into the output terminals (see ttg::send);
///           - `void(const keyT&, input_valuesT&&...)`: simplified form, with no type-checking of the dataflow into the output terminals.
/// @param[in] inedges a tuple of input edges
/// @param[in] outedges a tuple of output edges
/// @param[in] name a string label for the resulting TT
/// @param[in] name a string label for the resulting TT
/// @param[in] innames string labels for the respective input terminals of the resulting TT
/// @param[in] outnames string labels for the respective output terminals of the resulting TT
///
/// @warning You MUST NOT use generic callables that use concrete types for some data arguments, i.e. make either
///          ALL data types or NONE of them generic. This warning only applies to the data arguments and
///          does not apply to task ID (key) and optional out-terminal arguments.
///
/// @note For generic callables the arguments that are used read-only should be declared as `U&` (where `U` is the corresponding template parameter)
///       or `auto&` (in contexts such as generic lambdas where template arguments are implicit). The arguments that are
///       to be consumed (e.g. mutated, moved, etc.) should be declared as `U&&` or `auto&&` (i.e., as universal references).
///       For example, in
///       @code
///          make_tt([](auto& key, auto& datum1, auto&& datum2) { ... }, ...);
///       @endcode
///       the task id (`key`) and the first datum will be passed by const lvalue reference (i.e. no copy will be created by the runtime),
///       whereas the second datum will be passed by an rvalue reference, which may cause copying.
///       The corresponding free function analog of the above lambda is:
///       @code
///          template <typename K, typename D1, typename D2>
///          void func (K& key, D1& datum1, D2&& datum2) { ... }
///       @endcode
///
/// @warning Although generic arguments annotated by `const auto&` are also permitted, their use is discouraged to avoid confusion;
///          namely, `const auto&` denotes a _consumable_ argument, NOT read-only, despite the `const`.
// clang-format on
template <ttg::ExecutionSpace space, typename keyT = void, typename funcT,
          typename... input_edge_valuesT, typename... output_edgesT>
auto make_tt(funcT &&func,
             const std::tuple<ttg::Edge<keyT, input_edge_valuesT>...> &inedges =
                 std::tuple<>{},
             const std::tuple<output_edgesT...> &outedges = std::tuple<>{},
             const std::string &name = "wrapper",
             const std::vector<std::string> &innames = std::vector<std::string>(
                 sizeof...(input_edge_valuesT), "input"),
             const std::vector<std::string> &outnames =
                 std::vector<std::string>(sizeof...(output_edgesT), "output")) {
  // ensure input types do not contain Void
  static_assert(ttg::meta::is_none_Void_v<input_edge_valuesT...>,
                "ttg::Void is for internal use only, do not use it");

  using output_terminals_type = typename ttg::edges_to_output_terminals<
      std::tuple<output_edgesT...>>::type;

  constexpr auto void_key = ttg::meta::is_void_v<keyT>;

  // list of base datum types (T or const T)
  using base_input_data_t = ttg::meta::typelist<
      typename ttg::Edge<keyT, input_edge_valuesT>::value_type...>;

  // gross list of candidate argument types
  using gross_candidate_func_args_t = ttg::meta::typelist<
      ttg::meta::candidate_argument_bindings_t<std::add_const_t<keyT>>,
      ttg::meta::candidate_argument_bindings_t<
          typename ttg::Edge<keyT, input_edge_valuesT>::value_type>...,
      ttg::meta::typelist<output_terminals_type &, void>>;

  // net list of candidate argument types excludes the empty typelists for void
  // arguments
  using candidate_func_args_t =
      ttg::meta::filter_t<gross_candidate_func_args_t,
                          ttg::meta::typelist_is_not_empty>;

  // gross argument typelist for invoking func, can include void for optional
  // args
  constexpr static auto func_is_generic =
      ttg::meta::is_generic_callable_v<funcT>;
  using return_type_typelist_and_gross_func_args_t =
      decltype(ttg::meta::compute_arg_binding_types(func,
                                                    candidate_func_args_t{}));
  using func_return_t = std::tuple_element_t<
      0, std::tuple_element_t<0, return_type_typelist_and_gross_func_args_t>>;
  using gross_func_args_t =
      std::tuple_element_t<1, return_type_typelist_and_gross_func_args_t>;
  constexpr auto DETECTED_HOW_TO_INVOKE_GENERIC_FUNC =
      func_is_generic ? !std::is_same_v<gross_func_args_t, ttg::typelist<>>
                      : true;
  static_assert(
      DETECTED_HOW_TO_INVOKE_GENERIC_FUNC,
      "ttd::make_tt(func, inedges, ...): could not detect how to invoke "
      "generic callable func, either the "
      "signature of func "
      "is faulty, or inedges does match the expected list of types, or both");

  // net argument typelist
  using func_args_t = ttg::meta::drop_void_t<gross_func_args_t>;
  constexpr auto num_args = std::tuple_size_v<func_args_t>;

  // if given task id, make sure it's passed via const lvalue ref
  constexpr bool TASK_ID_PASSED_AS_CONST_LVALUE_REF =
      !void_key ? ttg::meta::probe_first_v<ttg::meta::is_const_lvalue_reference,
                                           true, func_args_t>
                : true;
  constexpr bool TASK_ID_PASSED_AS_NONREF =
      !void_key
          ? !ttg::meta::probe_first_v<std::is_reference, true, func_args_t>
          : true;
  static_assert(TASK_ID_PASSED_AS_CONST_LVALUE_REF || TASK_ID_PASSED_AS_NONREF,
                "ttg::make_tt(func, ...): if given to func, the task id must "
                "be passed by const lvalue ref or by value");

  // if given out-terminal tuple, make sure it's passed via nonconst lvalue ref
  constexpr bool have_outterm_tuple =
      func_is_generic
          ? !ttg::meta::is_last_void_v<gross_func_args_t>
          : ttg::meta::probe_last_v<ttg::meta::decays_to_output_terminal_tuple,
                                    false, gross_func_args_t>;
  constexpr bool OUTTERM_TUPLE_PASSED_AS_NONCONST_LVALUE_REF =
      have_outterm_tuple
          ? ttg::meta::probe_last_v<ttg::meta::is_nonconst_lvalue_reference,
                                    false, func_args_t>
          : true;
  static_assert(OUTTERM_TUPLE_PASSED_AS_NONCONST_LVALUE_REF,
                "ttg::make_tt(func, ...): if given to func, the output "
                "terminal tuple must be passed by nonconst lvalue ref");

  // TT needs actual types of arguments to func ... extract them and pass to
  // CallableWrapTTArgs
  using input_edge_value_types =
      ttg::meta::typelist<std::decay_t<input_edge_valuesT>...>;
  // input_args_t = {input_valuesT&&...}
  using input_args_t = typename ttg::meta::take_first_n<
      typename ttg::meta::drop_first_n<func_args_t,
                                       std::size_t(void_key ? 0 : 1)>::type,
      std::tuple_size_v<func_args_t> - (void_key ? 0 : 1) -
          (have_outterm_tuple ? 1 : 0)>::type;
  constexpr auto NO_ARGUMENTS_PASSED_AS_NONCONST_LVALUE_REF =
      !ttg::meta::is_any_nonconst_lvalue_reference_v<input_args_t>;
  static_assert(NO_ARGUMENTS_PASSED_AS_NONCONST_LVALUE_REF,
                "ttg::make_tt(func, inedges, outedges): one or more arguments "
                "to func can only be passed by nonconst lvalue "
                "ref; this is illegal, should only pass arguments as const "
                "lvalue ref or (nonconst) rvalue ref");
  using decayed_input_args_t = ttg::meta::decayed_typelist_t<input_args_t>;
  // 3. full_input_args_t = edge-types with non-void types replaced by
  // input_args_t
  using full_input_args_t =
      ttg::meta::replace_nonvoid_t<input_edge_value_types, input_args_t>;
  using wrapT = typename CallableWrapTTArgsAsTypelist<
      funcT, func_return_t, have_outterm_tuple, space, keyT,
      output_terminals_type, full_input_args_t>::type;

  return std::make_unique<wrapT>(std::forward<funcT>(func), inedges, outedges,
                                 name, innames, outnames);
}

template <typename keyT = void, typename funcT, typename... input_edge_valuesT,
          typename... output_edgesT>
auto make_tt(funcT &&func,
             const std::tuple<ttg::Edge<keyT, input_edge_valuesT>...> &inedges =
                 std::tuple<>{},
             const std::tuple<output_edgesT...> &outedges = std::tuple<>{},
             const std::string &name = "wrapper",
             const std::vector<std::string> &innames = std::vector<std::string>(
                 sizeof...(input_edge_valuesT), "input"),
             const std::vector<std::string> &outnames =
                 std::vector<std::string>(sizeof...(output_edgesT), "output")) {
  return make_tt<ttg::ExecutionSpace::Host>(std::forward<funcT>(func), inedges,
                                            outedges, name, innames, outnames);
}

template <typename keyT, typename funcT, typename... input_valuesT,
          typename... output_edgesT>
[[deprecated("use make_tt_tpl instead")]] inline auto wrapt(
    funcT &&func, const std::tuple<ttg::Edge<keyT, input_valuesT>...> &inedges,
    const std::tuple<output_edgesT...> &outedges,
    const std::string &name = "wrapper",
    const std::vector<std::string> &innames =
        std::vector<std::string>(sizeof...(input_valuesT), "input"),
    const std::vector<std::string> &outnames =
        std::vector<std::string>(sizeof...(output_edgesT), "output")) {
  return make_tt_tpl<keyT>(std::forward<funcT>(func), inedges, outedges, name,
                           innames, outnames);
}

template <typename keyT, typename funcT, typename... input_edge_valuesT,
          typename... output_edgesT>
[[deprecated("use make_tt instead")]] auto wrap(
    funcT &&func,
    const std::tuple<ttg::Edge<keyT, input_edge_valuesT>...> &inedges,
    const std::tuple<output_edgesT...> &outedges,
    const std::string &name = "wrapper",
    const std::vector<std::string> &innames =
        std::vector<std::string>(sizeof...(input_edge_valuesT), "input"),
    const std::vector<std::string> &outnames =
        std::vector<std::string>(sizeof...(output_edgesT), "output")) {
  return make_tt<keyT>(std::forward<funcT>(func), inedges, outedges, name,
                       innames, outnames);
}

// #include "ttg/make_device_tt.h"

#endif  // TTG_MAKE_TT_H

///////////////// REAL WORLD EXAMPLE 2: Libint2's engine.impl.h

/*
 *  Copyright (C) 2004-2021 Edward F. Valeev
 *
 *  This file is part of Libint.
 *
 *  Libint is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Libint is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with Libint.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _libint2_src_lib_libint_engineimpl_h_
#define _libint2_src_lib_libint_engineimpl_h_

#include <iterator>

#include "./deriv_map.h"
#include "./engine.h"

#pragma GCC diagnostic push
#pragma GCC system_header
#include <Eigen/Core>
#pragma GCC diagnostic pop

#include <libint2/boys.h>
#if LIBINT_HAS_SYSTEM_BOOST_PREPROCESSOR_VARIADICS
#include <boost/preprocessor.hpp>
#include <boost/preprocessor/facilities/is_1.hpp>
#else  // use bundled boost
#include <libint2/boost/preprocessor.hpp>
#include <libint2/boost/preprocessor/facilities/is_1.hpp>
#endif

// extra PP macros

#define BOOST_PP_MAKE_TUPLE_INTERNAL(z, i, last) \
  i BOOST_PP_COMMA_IF(BOOST_PP_NOT_EQUAL(i, last))
/// BOOST_PP_MAKE_TUPLE(n) returns (0,1,....n-1)
#define BOOST_PP_MAKE_TUPLE(n) \
  (BOOST_PP_REPEAT(n, BOOST_PP_MAKE_TUPLE_INTERNAL, BOOST_PP_DEC(n)))

// the engine will be profiled by default if library was configured with
// --enable-profile
#ifdef LIBINT2_PROFILE
#define LIBINT2_ENGINE_TIMERS
// uncomment if want to profile each integral class
#define LIBINT2_ENGINE_PROFILE_CLASS
#endif
// uncomment if want to profile the engine even if library was configured
// without --enable-profile
// #  define LIBINT2_ENGINE_TIMERS

namespace libint2 {

template <typename T, unsigned N>
typename std::remove_all_extents<T>::type *to_ptr1(T (&a)[N]) {
  return reinterpret_cast<typename std::remove_all_extents<T>::type *>(&a);
}

/// list of libint task names for each Operator type.
/// These MUST appear in the same order as in Operator.
/// You must also update BOOST_PP_NBODY_OPERATOR_LAST_ONEBODY_INDEX when you add
/// one-body ints
#define BOOST_PP_NBODY_OPERATOR_LIST \
  (overlap,                          \
   (kinetic,                         \
    (elecpot,                        \
     (elecpot,                       \
      (elecpot,                      \
       (1emultipole,                 \
        (2emultipole,                \
         (3emultipole,               \
          (sphemultipole,            \
           (eri,                     \
            (eri,                    \
             (eri,                   \
              (eri,                  \
               (eri,                 \
                (eri, (eri, (eri, (eri, (eri, BOOST_PP_NIL)))))))))))))))))))

#define BOOST_PP_NBODY_OPERATOR_INDEX_TUPLE \
  BOOST_PP_MAKE_TUPLE(BOOST_PP_LIST_SIZE(BOOST_PP_NBODY_OPERATOR_LIST))
#define BOOST_PP_NBODY_OPERATOR_INDEX_LIST \
  BOOST_PP_TUPLE_TO_LIST(BOOST_PP_NBODY_OPERATOR_INDEX_TUPLE)
#define BOOST_PP_NBODY_OPERATOR_LAST_ONEBODY_INDEX \
  8  // sphemultipole, the 9th member of BOOST_PP_NBODY_OPERATOR_LIST, is the
     // last 1-body operator

// make list of braket indices for n-body ints
#define BOOST_PP_NBODY_BRAKET_INDEX_TUPLE \
  BOOST_PP_MAKE_TUPLE(BOOST_PP_INC(BOOST_PP_NBODY_BRAKET_MAX_INDEX))
#define BOOST_PP_NBODY_BRAKET_INDEX_LIST \
  BOOST_PP_TUPLE_TO_LIST(BOOST_PP_NBODY_BRAKET_INDEX_TUPLE)
#define BOOST_PP_NBODY_BRAKET_RANK_TUPLE (2, 3, 4)
#define BOOST_PP_NBODY_BRAKET_RANK_LIST \
  BOOST_PP_TUPLE_TO_LIST(BOOST_PP_NBODY_BRAKET_RANK_TUPLE)

// make list of derivative orders for n-body ints
#define BOOST_PP_NBODY_DERIV_ORDER_TUPLE \
  BOOST_PP_MAKE_TUPLE(BOOST_PP_INC(LIBINT2_MAX_DERIV_ORDER))
#define BOOST_PP_NBODY_DERIV_ORDER_LIST \
  BOOST_PP_TUPLE_TO_LIST(BOOST_PP_NBODY_DERIV_ORDER_TUPLE)

/// the runtime version of \c operator_traits<oper>::default_params()
__libint2_engine_inline libint2::any default_params(const Operator &oper) {
  switch (static_cast<int>(oper)) {
#define BOOST_PP_NBODYENGINE_MCR1(r, data, i, elem) \
  case i:                                           \
    return operator_traits<static_cast<Operator>(i)>::default_params();
    BOOST_PP_LIST_FOR_EACH_I(BOOST_PP_NBODYENGINE_MCR1, _,
                             BOOST_PP_NBODY_OPERATOR_LIST)
    default:
      break;
  }
  assert(false && "missing case in switch");  // unreachable
  abort();
}

/// Computes target shell sets of integrals.

/// @return vector of pointers to target shell sets, the number of sets =
/// Engine::nshellsets();
///         if the first pointer equals \c nullptr then all elements were
///         screened out.
/// \note resulting shell sets are stored in row-major order.
/// \note Call Engine::compute1() or Engine::compute2() directly to avoid extra
/// copies.
template <typename... ShellPack>
__libint2_engine_inline const Engine::target_ptr_vec &Engine::compute(
    const libint2::Shell &first_shell, const ShellPack &...rest_of_shells) {
  constexpr auto nargs = 1 + sizeof...(rest_of_shells);
  assert(nargs == braket_rank() &&
         "# of arguments to compute() does not match the braket type");

  std::array<std::reference_wrapper<const Shell>, nargs> shells{
      {first_shell, rest_of_shells...}};

  if (operator_rank() == 1) {
    if (nargs == 2) return compute1(shells[0], shells[1]);
  } else if (operator_rank() == 2) {
    auto compute_ptr_idx = ((static_cast<int>(oper_) -
                             static_cast<int>(Operator::first_2body_oper)) *
                                nbrakets_2body +
                            (static_cast<int>(braket_) -
                             static_cast<int>(BraKet::first_2body_braket))) *
                               nderivorders_2body +
                           deriv_order_;
    assert(compute_ptr_idx >= 0 && compute_ptr_idx < compute2_ptrs().size());
    auto compute_ptr = compute2_ptrs()[compute_ptr_idx];
    assert(compute_ptr != nullptr && "2-body compute function not found");
    if (nargs == 2)
      return (this->*compute_ptr)(shells[0], Shell::unit(), shells[1],
                                  Shell::unit(), nullptr, nullptr);
    if (nargs == 3)
      return (this->*compute_ptr)(shells[0], Shell::unit(), shells[1],
                                  shells[2], nullptr, nullptr);
    if (nargs == 4)
      return (this->*compute_ptr)(shells[0], shells[1], shells[2], shells[3],
                                  nullptr, nullptr);
  }

  assert(false && "missing feature");  // only reached if missing a feature
  abort();
}

/// Computes target shell sets of 1-body integrals.
/// @return vector of pointers to target shell sets, the number of sets =
/// Engine::nshellsets()
/// \note resulting shell sets are stored in row-major order
__libint2_engine_inline const Engine::target_ptr_vec &Engine::compute1(
    const libint2::Shell &s1, const libint2::Shell &s2) {
  // can only handle 1 contraction at a time
  assert((s1.ncontr() == 1 && s2.ncontr() == 1) &&
         "generally-contracted shells not yet supported");

  const auto oper_is_nuclear =
      (oper_ == Operator::nuclear || oper_ == Operator::erf_nuclear ||
       oper_ == Operator::erfc_nuclear);

  const auto l1 = s1.contr[0].l;
  const auto l2 = s2.contr[0].l;
  assert(l1 <= lmax_ && "the angular momentum limit is exceeded");
  assert(l2 <= lmax_ && "the angular momentum limit is exceeded");

  // if want nuclear, make sure there is at least one nucleus .. otherwise the
  // user likely forgot to call set_params
  if (oper_is_nuclear && nparams() == 0)
    throw std::logic_error(
        "Engine<*nuclear>, but no charges found; forgot to call "
        "set_params()?");

  const auto n1 = s1.size();
  const auto n2 = s2.size();
  const auto n12 = n1 * n2;
  const auto ncart1 = s1.cartesian_size();
  const auto ncart2 = s2.cartesian_size();
  const auto ncart12 = ncart1 * ncart2;

  // assert # of primitive pairs
  const auto nprim1 = s1.nprim();
  const auto nprim2 = s2.nprim();
  const auto nprimpairs = nprim1 * nprim2;
  assert(nprimpairs <= primdata_.size() &&
         "the max number of primitive pairs exceeded");

  auto nparam_sets = nparams();

  // keep track if need to set targets_ explicitly
  bool set_targets = set_targets_;

  // # of targets computed by libint
  const auto ntargets = nopers() * num_geometrical_derivatives(2, deriv_order_);

  // Libint computes derivatives with respect to basis functions only, must
  // must use translational invariance to recover derivatives w.r.t. operator
  // degrees of freedom
  // will compute derivs w.r.t. 2 Gaussian centers + (if nuclear) nparam_sets
  // operator centers
  const auto nderivcenters_shset = 2 + (oper_is_nuclear ? nparam_sets : 0);
  const auto nderivcoord = 3 * nderivcenters_shset;
  const auto num_shellsets_computed =
      nopers() * num_geometrical_derivatives(nderivcenters_shset, deriv_order_);

  // will use scratch_ if:
  // - Coulomb ints are computed 1 charge at a time, contributions are
  // accumulated in scratch_ (unless la==lb==0)
  // - derivatives on the missing center need to be reconstructed (no need to
  // accumulate into scratch though)
  // NB ints in scratch are packed in order
  const auto accumulate_ints_in_scratch = oper_is_nuclear;

  // adjust max angular momentum, if needed
  const auto lmax = std::max(l1, l2);
  assert(lmax <= lmax_ && "the angular momentum limit is exceeded");

  // N.B. for l=0 no need to transform to solid harmonics
  // this is a workaround for the corner case of oper_ == Operator::*nuclear,
  // and solid harmonics (s|s) integral ... beware the integral storage state
  // machine
  const auto tform_to_solids =
      (s1.contr[0].pure || s2.contr[0].pure) && lmax != 0;

  // simple (s|s) ints will be computed directly and accumulated in the first
  // element of stack
  const auto compute_directly = lmax == 0 && deriv_order_ == 0 &&
                                (oper_ == Operator::overlap || oper_is_nuclear);
  if (compute_directly) {
    primdata_[0].stack[0] = 0;
    targets_[0] = primdata_[0].stack;
  }

  if (accumulate_ints_in_scratch)
    std::fill(std::begin(scratch_),
              std::begin(scratch_) + num_shellsets_computed * ncart12, 0.0);

  // loop over accumulation batches
  for (auto pset = 0u; pset != nparam_sets; ++pset) {
    if (!oper_is_nuclear)
      assert(nparam_sets == 1 && "unexpected number of operator parameters");

    auto p12 = 0;
    for (auto p1 = 0; p1 != nprim1; ++p1) {
      for (auto p2 = 0; p2 != nprim2; ++p2, ++p12) {
        compute_primdata(primdata_[p12], s1, s2, p1, p2, pset);
      }
    }
    primdata_[0].contrdepth = p12;

    if (compute_directly) {
      auto &result = primdata_[0].stack[0];
      switch (oper_) {
        case Operator::overlap:
          for (auto p12 = 0; p12 != primdata_[0].contrdepth; ++p12)
            result += primdata_[p12]._0_Overlap_0_x[0] *
                      primdata_[p12]._0_Overlap_0_y[0] *
                      primdata_[p12]._0_Overlap_0_z[0];
          break;
        case Operator::nuclear:
        case Operator::erf_nuclear:
        case Operator::erfc_nuclear:
          for (auto p12 = 0; p12 != primdata_[0].contrdepth; ++p12)
            result += primdata_[p12].LIBINT_T_S_ELECPOT_S(0)[0];
          break;
        default:
          assert(false && "missing case in switch");
      }
      primdata_[0].targets[0] = &result;
    } else {
      const auto buildfnidx = s1.contr[0].l * hard_lmax_ + s2.contr[0].l;
      assert(buildfnptrs_[buildfnidx] && "null build function ptr");
      buildfnptrs_[buildfnidx](&primdata_[0]);

      if (accumulate_ints_in_scratch) {
        set_targets = true;
        // - for non-derivative ints and first derivative ints the target
        //   ints computed by libint will appear at the front of targets_
        // - for second and higher derivs need to re-index targets, hence
        //   will accumulate later, when computing operator derivatives via
        //   transinv
        if (deriv_order_ <= 1) {
          // accumulate targets computed by libint for this pset into the
          // accumulated targets in scratch
          auto s_target = &scratch_[0];
          for (auto s = 0; s != ntargets; ++s, s_target += ncart12)
            if (pset != 0)
              std::transform(primdata_[0].targets[s],
                             primdata_[0].targets[s] + ncart12, s_target,
                             s_target, std::plus<value_type>());
            else
              std::copy(primdata_[0].targets[s],
                        primdata_[0].targets[s] + ncart12, s_target);
        }

        // 2. reconstruct derivatives of nuclear ints for each nucleus
        //    using translational invariance
        // NB this is done in cartesian basis, otherwise would have to tform
        // to solids contributions from every atom, rather than the running
        // total at the end
        if (deriv_order_ > 0) {
          switch (deriv_order_) {
            case 1: {
              // first 6 shellsets are derivatives with respect to Gaussian
              // positions
              // following them are derivs with respect to nuclear coordinates
              // (3 per nucleus)
              assert(ntargets == 6 && "unexpected # of targets");
              auto dest = &scratch_[0] + (6 + pset * 3) * ncart12;
              for (auto s = 0; s != 3; ++s, dest += ncart12) {
                auto src = primdata_[0].targets[s];
                for (auto i = 0; i != ncart12; ++i) {
                  dest[i] = -src[i];
                }
              }
              dest -= 3 * ncart12;
              for (auto s = 3; s != 6; ++s, dest += ncart12) {
                auto src = primdata_[0].targets[s];
                for (auto i = 0; i != ncart12; ++i) {
                  dest[i] -= src[i];
                }
              }
            } break;

            case 2: {
              // computes upper triangle index
              // n2 = matrix size times 2
              // i,j = indices, i<j
              auto upper_triangle_index_ord = [](int n2, int i, int j) {
                return i * (n2 - i - 1) / 2 + j;
              };
              // same as above, but orders i and j
              auto upper_triangle_index = [&](int n2, int i, int j) {
                return upper_triangle_index_ord(n2, std::min(i, j),
                                                std::max(i, j));
              };

              // accumulate ints for this pset to scratch in locations
              // remapped to overall deriv index
              const auto ncoords_times_two = nderivcoord * 2;
              for (auto d0 = 0, d01 = 0; d0 != 6; ++d0) {
                for (auto d1 = d0; d1 != 6; ++d1, ++d01) {
                  const auto d01_full =
                      upper_triangle_index_ord(ncoords_times_two, d0, d1);
                  auto tgt = &scratch_[d01_full * ncart12];
                  if (pset != 0)
                    std::transform(primdata_[0].targets[d01],
                                   primdata_[0].targets[d01] + ncart12, tgt,
                                   tgt, std::plus<value_type>());
                  else
                    std::copy(primdata_[0].targets[d01],
                              primdata_[0].targets[d01] + ncart12, tgt);
                }
              }

              // use translational invariance to build derivatives w.r.t.
              // operator centers
              {
                // mixed derivatives: first deriv w.r.t. Gaussian, second
                // w.r.t. operator coord pset
                const auto c1 = 2 + pset;
                for (auto c0 = 0; c0 != 2; ++c0) {
                  for (auto xyz0 = 0; xyz0 != 3; ++xyz0) {
                    const auto coord0 = c0 * 3 + xyz0;
                    for (auto xyz1 = 0; xyz1 != 3; ++xyz1) {
                      const auto coord1 = c1 * 3 + xyz1;  // coord1 > coord0

                      const auto coord01_abs = upper_triangle_index_ord(
                          ncoords_times_two, coord0, coord1);
                      auto tgt = &scratch_[coord01_abs * ncart12];

                      // d2 / dAi dOj = - d2 / dAi dAj
                      {
                        auto coord1_A = xyz1;
                        const auto coord01_A =
                            upper_triangle_index(12, coord0, coord1_A);
                        const auto src = primdata_[0].targets[coord01_A];
                        for (auto i = 0; i != ncart12; ++i) tgt[i] = -src[i];
                      }

                      // d2 / dAi dOj -= d2 / dAi dBj
                      {
                        auto coord1_B = 3 + xyz1;
                        const auto coord01_B =
                            upper_triangle_index(12, coord0, coord1_B);
                        const auto src = primdata_[0].targets[coord01_B];
                        for (auto i = 0; i != ncart12; ++i) tgt[i] -= src[i];
                      }
                    }
                  }
                }
              }  // mixed derivs
              {
                // operator derivs
                const auto c0 = 2 + pset;
                const auto c1 = c0;
                for (auto xyz0 = 0; xyz0 != 3; ++xyz0) {
                  const auto coord0 = c0 * 3 + xyz0;
                  for (auto xyz1 = xyz0; xyz1 != 3; ++xyz1) {
                    const auto coord1 = c1 * 3 + xyz1;  // coord1 > coord0

                    const auto coord01_abs = upper_triangle_index_ord(
                        ncoords_times_two, coord0, coord1);
                    auto tgt = &scratch_[coord01_abs * ncart12];

                    // d2 / dOi dOj = d2 / dAi dAj
                    {
                      auto coord0_A = xyz0;
                      auto coord1_A = xyz1;
                      const auto coord01_AA =
                          upper_triangle_index_ord(12, coord0_A, coord1_A);
                      const auto src = primdata_[0].targets[coord01_AA];
                      for (auto i = 0; i != ncart12; ++i) tgt[i] = src[i];
                    }

                    // d2 / dOi dOj += d2 / dAi dBj
                    {
                      auto coord0_A = xyz0;
                      auto coord1_B = 3 + xyz1;
                      const auto coord01_AB =
                          upper_triangle_index_ord(12, coord0_A, coord1_B);
                      const auto src = primdata_[0].targets[coord01_AB];
                      for (auto i = 0; i != ncart12; ++i) tgt[i] += src[i];
                    }

                    // d2 / dOi dOj += d2 / dBi dAj
                    {
                      auto coord0_B = 3 + xyz0;
                      auto coord1_A = xyz1;
                      const auto coord01_BA =
                          upper_triangle_index_ord(12, coord1_A, coord0_B);
                      const auto src = primdata_[0].targets[coord01_BA];
                      for (auto i = 0; i != ncart12; ++i) tgt[i] += src[i];
                    }

                    // d2 / dOi dOj += d2 / dBi dBj
                    {
                      auto coord0_B = 3 + xyz0;
                      auto coord1_B = 3 + xyz1;
                      const auto coord01_BB =
                          upper_triangle_index_ord(12, coord0_B, coord1_B);
                      const auto src = primdata_[0].targets[coord01_BB];
                      for (auto i = 0; i != ncart12; ++i) tgt[i] += src[i];
                    }
                  }
                }
              }  // operator derivs
            } break;

            default: {
              assert(deriv_order_ <= 2 && "feature not implemented");

              // 1. since # of derivatives changes, remap derivatives computed
              //    by libint; targets_ will hold the "remapped" pointers to
              //    the data
              using ShellSetDerivIterator =
                  libint2::FixedOrderedIntegerPartitionIterator<
                      std::vector<unsigned int>>;
              ShellSetDerivIterator shellset_gaussian_diter(deriv_order_, 2);
              ShellSetDerivIterator shellset_full_diter(deriv_order_,
                                                        nderivcenters_shset);
              std::vector<unsigned int> full_deriv(3 * nderivcenters_shset, 0);
              std::size_t s = 0;
              while (shellset_gaussian_diter) {  // loop over derivs computed
                // by libint
                const auto &s1s2_deriv = *shellset_gaussian_diter;
                std::copy(std::begin(s1s2_deriv), std::end(s1s2_deriv),
                          std::begin(full_deriv));
                const auto full_rank = ShellSetDerivIterator::rank(full_deriv);
                targets_[full_rank] = primdata_[0].targets[s];
              }
              // use translational invariance to build derivatives w.r.t.
              // operator centers
            }

          }  // deriv_order_ switch
        }    // reconstruct derivatives
      }
    }  // ltot != 0
  }    // pset (accumulation batches)

  if (tform_to_solids) {
    set_targets = false;
    // where do spherical ints go?
    auto *spherical_ints =
        (accumulate_ints_in_scratch) ? scratch2_ : &scratch_[0];

    // transform to solid harmonics, one shell set at a time:
    // for each computed shell set ...
    for (auto s = 0ul; s != num_shellsets_computed;
         ++s, spherical_ints += n12) {
      auto cartesian_ints = accumulate_ints_in_scratch
                                ? &scratch_[s * ncart12]
                                : primdata_[0].targets[s];
      // transform
      if (s1.contr[0].pure && s2.contr[0].pure) {
        libint2::solidharmonics::tform(l1, l2, cartesian_ints, spherical_ints);
      } else {
        if (s1.contr[0].pure)
          libint2::solidharmonics::tform_rows(l1, n2, cartesian_ints,
                                              spherical_ints);
        else
          libint2::solidharmonics::tform_cols(n1, l2, cartesian_ints,
                                              spherical_ints);
      }
      // .. and compute the destination
      targets_[s] = spherical_ints;
    }  // loop cartesian shell set
  }    // tform to solids

  if (set_targets) {
    for (auto s = 0ul; s != num_shellsets_computed; ++s) {
      auto cartesian_ints = accumulate_ints_in_scratch
                                ? &scratch_[s * ncart12]
                                : primdata_[0].targets[s];
      targets_[s] = cartesian_ints;
    }
  }

  if (cartesian_shell_normalization() == CartesianShellNormalization::uniform) {
    std::array<std::reference_wrapper<const Shell>, 2> shells{s1, s2};
    for (auto s = 0ul; s != num_shellsets_computed; ++s) {
      uniform_normalize_cartesian_shells(const_cast<value_type *>(targets_[s]),
                                         shells);
    }
  }

  return targets_;
}

// generic _initializer
__libint2_engine_inline void Engine::_initialize() {
#define BOOST_PP_NBODYENGINE_MCR3_ncenter(product) \
  BOOST_PP_TUPLE_ELEM(3, 1, product)

#define BOOST_PP_NBODYENGINE_MCR3_default_ncenter(product)                   \
  BOOST_PP_IIF(BOOST_PP_GREATER(BOOST_PP_TUPLE_ELEM(3, 0, product),          \
                                BOOST_PP_NBODY_OPERATOR_LAST_ONEBODY_INDEX), \
               4, 2)

#define BOOST_PP_NBODYENGINE_MCR3_NCENTER(product)                            \
  BOOST_PP_IIF(                                                               \
      BOOST_PP_NOT_EQUAL(BOOST_PP_NBODYENGINE_MCR3_ncenter(product),          \
                         BOOST_PP_NBODYENGINE_MCR3_default_ncenter(product)), \
      BOOST_PP_NBODYENGINE_MCR3_ncenter(product), BOOST_PP_EMPTY())

#define BOOST_PP_NBODYENGINE_MCR3_OPER(product)  \
  BOOST_PP_LIST_AT(BOOST_PP_NBODY_OPERATOR_LIST, \
                   BOOST_PP_TUPLE_ELEM(3, 0, product))

#define BOOST_PP_NBODYENGINE_MCR3_DERIV(product)                        \
  BOOST_PP_IIF(BOOST_PP_GREATER(BOOST_PP_TUPLE_ELEM(3, 2, product), 0), \
               BOOST_PP_TUPLE_ELEM(3, 2, product), BOOST_PP_EMPTY())

#define BOOST_PP_NBODYENGINE_MCR3_task(product)                         \
  BOOST_PP_CAT(BOOST_PP_CAT(BOOST_PP_NBODYENGINE_MCR3_ncenter(product), \
                            BOOST_PP_NBODYENGINE_MCR3_OPER(product)),   \
               BOOST_PP_NBODYENGINE_MCR3_DERIV(product))

#define BOOST_PP_NBODYENGINE_MCR3_TASK(product)                             \
  BOOST_PP_IIF(                                                             \
      BOOST_PP_CAT(LIBINT2_TASK_EXISTS_,                                    \
                   BOOST_PP_NBODYENGINE_MCR3_task(product)),                \
      BOOST_PP_CAT(BOOST_PP_CAT(BOOST_PP_NBODYENGINE_MCR3_NCENTER(product), \
                                BOOST_PP_NBODYENGINE_MCR3_OPER(product)),   \
                   BOOST_PP_NBODYENGINE_MCR3_DERIV(product)),               \
      default)

#define BOOST_PP_NBODYENGINE_MCR3(r, product)                                  \
  if (static_cast<int>(oper_) == BOOST_PP_TUPLE_ELEM(3, 0, product) &&         \
      static_cast<int>(rank(braket_)) == BOOST_PP_TUPLE_ELEM(3, 1, product) && \
      deriv_order_ == BOOST_PP_TUPLE_ELEM(3, 2, product)) {                    \
    hard_lmax_ = BOOST_PP_CAT(LIBINT2_MAX_AM_,                                 \
                              BOOST_PP_NBODYENGINE_MCR3_TASK(product)) +       \
                 1;                                                            \
    hard_default_lmax_ = BOOST_PP_IF(                                          \
        BOOST_PP_IS_1(BOOST_PP_CAT(LIBINT2_CENTER_DEPENDENT_MAX_AM_,           \
                                   BOOST_PP_NBODYENGINE_MCR3_task(product))),  \
        BOOST_PP_CAT(                                                          \
            LIBINT2_MAX_AM_,                                                   \
            BOOST_PP_CAT(default, BOOST_PP_NBODYENGINE_MCR3_DERIV(product))) + \
            1,                                                                 \
        std::numeric_limits<int>::max());                                      \
    const auto lmax = BOOST_PP_IF(                                             \
        BOOST_PP_IS_1(BOOST_PP_CAT(LIBINT2_CENTER_DEPENDENT_MAX_AM_,           \
                                   BOOST_PP_NBODYENGINE_MCR3_task(product))),  \
        std::max(hard_lmax_, hard_default_lmax_), hard_lmax_);                 \
    if (lmax_ >= lmax) {                                                       \
      throw Engine::lmax_exceeded(BOOST_PP_STRINGIZE(BOOST_PP_NBODYENGINE_MCR3_TASK(product)), lmax, lmax_);              \
    }                                                                          \
    if (stack_size_ > 0) libint2_cleanup_default(&primdata_[0]);               \
    stack_size_ = LIBINT2_PREFIXED_NAME(BOOST_PP_CAT(                          \
        libint2_need_memory_, BOOST_PP_NBODYENGINE_MCR3_TASK(product)))(       \
        lmax_);                                                                \
    LIBINT2_PREFIXED_NAME(                                                     \
        BOOST_PP_CAT(libint2_init_, BOOST_PP_NBODYENGINE_MCR3_TASK(product)))  \
    (&primdata_[0], lmax_, 0);                                                 \
    BOOST_PP_IF(BOOST_PP_IS_1(LIBINT2_FLOP_COUNT),                             \
                LIBINT2_PREFIXED_NAME(libint2_init_flopcounter)(               \
                    &primdata_[0], primdata_.size()),                          \
                BOOST_PP_EMPTY());                                             \
    buildfnptrs_ = to_ptr1(LIBINT2_PREFIXED_NAME(BOOST_PP_CAT(                 \
        libint2_build_, BOOST_PP_NBODYENGINE_MCR3_TASK(product))));            \
    reset_scratch();                                                           \
    return;                                                                    \
  }

  BOOST_PP_LIST_FOR_EACH_PRODUCT(
      BOOST_PP_NBODYENGINE_MCR3, 3,
      (BOOST_PP_NBODY_OPERATOR_INDEX_LIST, BOOST_PP_NBODY_BRAKET_RANK_LIST,
       BOOST_PP_NBODY_DERIV_ORDER_LIST))

  assert(false &&
         "missing case in switch");  // either deriv_order_ or oper_ is wrong
  abort();
}  // _initialize<R>()

__libint2_engine_inline void Engine::initialize(size_t max_nprim) {
  assert(libint2::initialized() && "libint is not initialized");
  assert(deriv_order_ <= LIBINT2_MAX_DERIV_ORDER &&
         "exceeded the max derivative order of the library");

  // validate braket
#ifndef INCLUDE_ONEBODY
  assert(braket_ != BraKet::x_x &&
         "this braket type not supported by the library; give --enable-1body "
         "to configure");
#endif
#ifndef INCLUDE_ERI
  assert(braket_ != BraKet::xx_xx &&
         "this braket type not supported by the library; give --enable-eri to "
         "configure");
#endif
#ifndef INCLUDE_ERI3
  assert((braket_ != BraKet::xs_xx && braket_ != BraKet::xx_xs) &&
         "this braket type not supported by the library; give --enable-eri3 to "
         "configure");
#endif
#ifndef INCLUDE_ERI2
  assert(braket_ != BraKet::xs_xs &&
         "this braket type not supported by the library; give --enable-eri2 to "
         "configure");
#endif

  // make sure it's no default initialized
  if (lmax_ < 0) throw using_default_initialized();

  // initialize braket, if needed
  if (braket_ == BraKet::invalid) braket_ = default_braket(oper_);

  if (max_nprim != 0) primdata_.resize(std::pow(max_nprim, braket_rank()));

  // initialize targets
  {
    decltype(targets_)::allocator_type alloc(primdata_[0].targets);
    targets_ = decltype(targets_)(alloc);
    // in some cases extra memory use can be avoided if targets_ manages its own
    // memory
    // the only instance is where we permute derivative integrals, this calls
    // for permuting
    // target indices.
    const auto permutable_targets =
        deriv_order_ > 0 &&
        (braket_ == BraKet::xx_xx || braket_ == BraKet::xs_xx ||
         braket_ == BraKet::xx_xs);
    if (permutable_targets)
      targets_.reserve(max_ntargets + 1);
    else
      targets_.reserve(max_ntargets);
    // will be resized to appropriate size in reset_scratch via _initialize
  }

#ifdef LIBINT2_ENGINE_TIMERS
  timers.set_now_overhead(25);
#endif
#ifdef LIBINT2_PROFILE
  primdata_[0].timers->set_now_overhead(25);
#endif

  _initialize();
}

namespace detail {
__libint2_engine_inline std::vector<Engine::compute2_ptr_type>
init_compute2_ptrs() {
  auto max_ncompute2_ptrs = nopers_2body * nbrakets_2body * nderivorders_2body;
  std::vector<Engine::compute2_ptr_type> result(max_ncompute2_ptrs, nullptr);

#define BOOST_PP_NBODYENGINE_MCR7(r, product)                                 \
  if (BOOST_PP_TUPLE_ELEM(3, 0, product) >=                                   \
          static_cast<int>(Operator::first_2body_oper) &&                     \
      BOOST_PP_TUPLE_ELEM(3, 0, product) <=                                   \
          static_cast<int>(Operator::last_2body_oper) &&                      \
      BOOST_PP_TUPLE_ELEM(3, 1, product) >=                                   \
          static_cast<int>(BraKet::first_2body_braket) &&                     \
      BOOST_PP_TUPLE_ELEM(3, 1, product) <=                                   \
          static_cast<int>(BraKet::last_2body_braket)) {                      \
    auto compute_ptr_idx = ((BOOST_PP_TUPLE_ELEM(3, 0, product) -             \
                             static_cast<int>(Operator::first_2body_oper)) *  \
                                nbrakets_2body +                              \
                            (BOOST_PP_TUPLE_ELEM(3, 1, product) -             \
                             static_cast<int>(BraKet::first_2body_braket))) * \
                               nderivorders_2body +                           \
                           BOOST_PP_TUPLE_ELEM(3, 2, product);                \
    result.at(compute_ptr_idx) = &Engine::compute2<                           \
        static_cast<Operator>(BOOST_PP_TUPLE_ELEM(3, 0, product)),            \
        static_cast<BraKet>(BOOST_PP_TUPLE_ELEM(3, 1, product)),              \
        BOOST_PP_TUPLE_ELEM(3, 2, product)>;                                  \
  }

  BOOST_PP_LIST_FOR_EACH_PRODUCT(
      BOOST_PP_NBODYENGINE_MCR7, 3,
      (BOOST_PP_NBODY_OPERATOR_INDEX_LIST, BOOST_PP_NBODY_BRAKET_INDEX_LIST,
       BOOST_PP_NBODY_DERIV_ORDER_LIST))

  return result;
}
}  // namespace detail

__libint2_engine_inline const std::vector<Engine::compute2_ptr_type> &
Engine::compute2_ptrs() const {
  static std::vector<compute2_ptr_type> compute2_ptrs_ =
      detail::init_compute2_ptrs();
  return compute2_ptrs_;
}

__libint2_engine_inline unsigned int Engine::nparams() const {
  switch (oper_) {
    case Operator::nuclear:
      return any_cast<
                 const operator_traits<Operator::nuclear>::oper_params_type &>(
                 params_)
          .size();
    case Operator::erf_nuclear:
    case Operator::erfc_nuclear:
      return std::get<1>(
                 any_cast<const operator_traits<
                     Operator::erfc_nuclear>::oper_params_type &>(params_))
          .size();
    default:
      return 1;
  }
  return 1;
}
__libint2_engine_inline unsigned int Engine::nopers() const {
  switch (static_cast<int>(oper_)) {
#define BOOST_PP_NBODYENGINE_MCR4(r, data, i, elem) \
  case i:                                           \
    return operator_traits<static_cast<Operator>(i)>::nopers;
    BOOST_PP_LIST_FOR_EACH_I(BOOST_PP_NBODYENGINE_MCR4, _,
                             BOOST_PP_NBODY_OPERATOR_LIST)
    default:
      break;
  }
  assert(false && "missing case in switch");  // unreachable
  abort();
}

template <>
__libint2_engine_inline any Engine::enforce_params_type<any>(
    Operator oper, const any &params, bool throw_if_wrong_type) {
  any result;
  switch (static_cast<int>(oper)) {
#define BOOST_PP_NBODYENGINE_MCR5A(r, data, i, elem)                           \
  case i:                                                                      \
    if (any_cast<operator_traits<static_cast<Operator>(i)>::oper_params_type>( \
            &params) != nullptr) {                                             \
      result = params;                                                         \
    } else {                                                                   \
      if (throw_if_wrong_type) throw bad_any_cast();                           \
      result = operator_traits<static_cast<Operator>(i)>::default_params();    \
    }                                                                          \
    break;

    BOOST_PP_LIST_FOR_EACH_I(BOOST_PP_NBODYENGINE_MCR5A, _,
                             BOOST_PP_NBODY_OPERATOR_LIST)

    default:
      assert(false && "missing case in switch");  // missed a case?
      abort();
  }
  return result;
}

template <typename Params>
__libint2_engine_inline any Engine::enforce_params_type(
    Operator oper, const Params &params, bool throw_if_wrong_type) {
  any result;
  switch (static_cast<int>(oper)) {
#define BOOST_PP_NBODYENGINE_MCR5B(r, data, i, elem)                        \
  case i:                                                                   \
    if (std::is_same<Params, operator_traits<static_cast<Operator>(         \
                                 i)>::oper_params_type>::value) {           \
      result = params;                                                      \
    } else {                                                                \
      if (throw_if_wrong_type) throw std::bad_cast();                       \
      result = operator_traits<static_cast<Operator>(i)>::default_params(); \
    }                                                                       \
    break;

    BOOST_PP_LIST_FOR_EACH_I(BOOST_PP_NBODYENGINE_MCR5B, _,
                             BOOST_PP_NBODY_OPERATOR_LIST)

    default:
      assert(false && "missing case in switch");  // missed a case?
      abort();
  }
  return result;
}

__libint2_engine_inline any Engine::make_core_eval_pack(Operator oper) const {
  any result;
  switch (static_cast<int>(oper)) {
#define BOOST_PP_NBODYENGINE_MCR6(r, data, i, elem)                          \
  case i:                                                                    \
    result = libint2::detail::make_compressed_pair(                          \
        operator_traits<static_cast<Operator>(i)>::core_eval_type::instance( \
            braket_rank() * lmax_ + deriv_order_,                            \
            std::numeric_limits<scalar_type>::epsilon()),                    \
        libint2::detail::CoreEvalScratch<                                    \
            operator_traits<static_cast<Operator>(i)>::core_eval_type>(      \
            braket_rank() * lmax_ + deriv_order_));                          \
    assert(any_cast<detail::core_eval_pack_type<static_cast<Operator>(i)>>(  \
               &result) != nullptr);                                         \
    break;

    BOOST_PP_LIST_FOR_EACH_I(BOOST_PP_NBODYENGINE_MCR6, _,
                             BOOST_PP_NBODY_OPERATOR_LIST)

    default:
      assert(false && "missing case in switch");  // missed a case?
      abort();
  }
  return result;
}

__libint2_engine_inline void Engine::init_core_ints_params(const any &params) {
  if (oper_ == Operator::delcgtg2) {
    // [g12,[- \Del^2, g12] = 2 (\Del g12) \cdot (\Del g12)
    // (\Del exp(-a r_12^2) \cdot (\Del exp(-b r_12^2) = 4 a b (r_{12}^2 exp(-
    // (a+b) r_{12}^2) )
    // i.e. need to scale each coefficient by 4 a b
    const auto &oparams =
        any_cast<const operator_traits<Operator::delcgtg2>::oper_params_type &>(
            params);
    const auto ng = oparams.size();
    operator_traits<Operator::delcgtg2>::oper_params_type core_ints_params;
    core_ints_params.reserve(ng * (ng + 1) / 2);
    for (size_t b = 0; b < ng; ++b)
      for (size_t k = 0; k <= b; ++k) {
        const auto gexp = oparams[b].first + oparams[k].first;
        const auto gcoeff = oparams[b].second * oparams[k].second *
                            (b == k ? 1 : 2);  // if a != b include ab and ba
        const auto gcoeff_rescaled =
            4 * oparams[b].first * oparams[k].first * gcoeff;
        core_ints_params.push_back(std::make_pair(gexp, gcoeff_rescaled));
      }
    core_ints_params_ = core_ints_params;
  } else {
    core_ints_params_ = params;
  }
}

__libint2_engine_inline void Engine::compute_primdata(Libint_t &primdata,
                                                      const Shell &s1,
                                                      const Shell &s2,
                                                      size_t p1, size_t p2,
                                                      size_t oset) {
  const auto &A = s1.O;
  const auto &B = s2.O;

  const auto alpha1 = s1.alpha[p1];
  const auto alpha2 = s2.alpha[p2];

  const auto c1 = s1.contr[0].coeff[p1];
  const auto c2 = s2.contr[0].coeff[p2];

  const auto gammap = alpha1 + alpha2;
  const auto oogammap = 1 / gammap;
  const auto rhop_over_alpha1 = alpha2 * oogammap;
  const auto rhop = alpha1 * rhop_over_alpha1;
  const auto Px = (alpha1 * A[0] + alpha2 * B[0]) * oogammap;
  const auto Py = (alpha1 * A[1] + alpha2 * B[1]) * oogammap;
  const auto Pz = (alpha1 * A[2] + alpha2 * B[2]) * oogammap;
  const auto AB_x = A[0] - B[0];
  const auto AB_y = A[1] - B[1];
  const auto AB_z = A[2] - B[2];
  const auto AB2_x = AB_x * AB_x;
  const auto AB2_y = AB_y * AB_y;
  const auto AB2_z = AB_z * AB_z;

  assert(LIBINT2_SHELLQUARTET_SET == LIBINT2_SHELLQUARTET_SET_STANDARD &&
         "non-standard shell ordering");

  const auto oper_is_nuclear =
      (oper_ == Operator::nuclear || oper_ == Operator::erf_nuclear ||
       oper_ == Operator::erfc_nuclear);

  // need to use HRR? see strategy.cc
  const auto l1 = s1.contr[0].l;
  const auto l2 = s2.contr[0].l;
  const bool use_hrr =
      (oper_is_nuclear || oper_ == Operator::sphemultipole) && l1 > 0 && l2 > 0;
  // unlike the 2-body ints, can go both ways, determine which way to go (the
  // logic must match TwoCenter_OS_Tactic)
  const bool hrr_ket_to_bra = l1 >= l2;
  if (use_hrr) {
    if (hrr_ket_to_bra) {
#if LIBINT2_DEFINED(eri, AB_x)
      primdata.AB_x[0] = AB_x;
#endif
#if LIBINT2_DEFINED(eri, AB_y)
      primdata.AB_y[0] = AB_y;
#endif
#if LIBINT2_DEFINED(eri, AB_z)
      primdata.AB_z[0] = AB_z;
#endif
    } else {
#if LIBINT2_DEFINED(eri, BA_x)
      primdata.BA_x[0] = -AB_x;
#endif
#if LIBINT2_DEFINED(eri, BA_y)
      primdata.BA_y[0] = -AB_y;
#endif
#if LIBINT2_DEFINED(eri, BA_z)
      primdata.BA_z[0] = -AB_z;
#endif
    }
  }

  // figure out whether will do VRR on center A and/or B
//  if ((!use_hrr && l1 > 0) || hrr_ket_to_bra) {
#if LIBINT2_DEFINED(eri, PA_x)
  primdata.PA_x[0] = Px - A[0];
#endif
#if LIBINT2_DEFINED(eri, PA_y)
  primdata.PA_y[0] = Py - A[1];
#endif
#if LIBINT2_DEFINED(eri, PA_z)
  primdata.PA_z[0] = Pz - A[2];
#endif
//  }
//
//  if ((!use_hrr && l2 > 0) || !hrr_ket_to_bra) {
#if LIBINT2_DEFINED(eri, PB_x)
  primdata.PB_x[0] = Px - B[0];
#endif
#if LIBINT2_DEFINED(eri, PB_y)
  primdata.PB_y[0] = Py - B[1];
#endif
#if LIBINT2_DEFINED(eri, PB_z)
  primdata.PB_z[0] = Pz - B[2];
#endif
  //  }

  if (oper_ == Operator::emultipole1 || oper_ == Operator::emultipole2 ||
      oper_ == Operator::emultipole3) {
    const auto &O = any_cast<
        const operator_traits<Operator::emultipole1>::oper_params_type &>(
        params_);  // same as emultipoleX
#if LIBINT2_DEFINED(eri, BO_x)
    primdata.BO_x[0] = B[0] - O[0];
#endif
#if LIBINT2_DEFINED(eri, BO_y)
    primdata.BO_y[0] = B[1] - O[1];
#endif
#if LIBINT2_DEFINED(eri, BO_z)
    primdata.BO_z[0] = B[2] - O[2];
#endif
  }
  if (oper_ == Operator::sphemultipole) {
    const auto &O = any_cast<
        const operator_traits<Operator::emultipole1>::oper_params_type &>(
        params_);
#if LIBINT2_DEFINED(eri, PO_x)
    primdata.PO_x[0] = Px - O[0];
#endif
#if LIBINT2_DEFINED(eri, PO_y)
    primdata.PO_y[0] = Py - O[1];
#endif
#if LIBINT2_DEFINED(eri, PO_z)
    primdata.PO_z[0] = Pz - O[2];
#endif
#if LIBINT2_DEFINED(eri, PO2)
    primdata.PO2[0] = (Px - O[0]) * (Px - O[0]) + (Py - O[1]) * (Py - O[1]) +
                      (Pz - O[2]) * (Pz - O[2]);
#endif
  }

#if LIBINT2_DEFINED(eri, oo2z)
  primdata.oo2z[0] = 0.5 * oogammap;
#endif

  decltype(c1) sqrt_PI(1.77245385090551602729816748334);
  const auto xyz_pfac = sqrt_PI * sqrt(oogammap);
  const auto ovlp_ss_x = exp(-rhop * AB2_x) * xyz_pfac * c1 * c2 * scale_;
  const auto ovlp_ss_y = exp(-rhop * AB2_y) * xyz_pfac;
  const auto ovlp_ss_z = exp(-rhop * AB2_z) * xyz_pfac;

  primdata._0_Overlap_0_x[0] = ovlp_ss_x;
  primdata._0_Overlap_0_y[0] = ovlp_ss_y;
  primdata._0_Overlap_0_z[0] = ovlp_ss_z;

  if (oper_ == Operator::kinetic || (deriv_order_ > 0)) {
#if LIBINT2_DEFINED(eri, two_alpha0_bra)
    primdata.two_alpha0_bra[0] = 2.0 * alpha1;
#endif
#if LIBINT2_DEFINED(eri, two_alpha0_ket)
    primdata.two_alpha0_ket[0] = 2.0 * alpha2;
#endif
  }

  if (oper_is_nuclear) {
    const auto &params =
        (oper_ == Operator::nuclear)
            ? any_cast<
                  const operator_traits<Operator::nuclear>::oper_params_type &>(
                  params_)
            : std::get<1>(
                  any_cast<const operator_traits<
                      Operator::erfc_nuclear>::oper_params_type &>(params_));

    const auto &C = params[oset].second;
    const auto &q = params[oset].first;
#if LIBINT2_DEFINED(eri, PC_x)
    primdata.PC_x[0] = Px - C[0];
#endif
#if LIBINT2_DEFINED(eri, PC_y)
    primdata.PC_y[0] = Py - C[1];
#endif
#if LIBINT2_DEFINED(eri, PC_z)
    primdata.PC_z[0] = Pz - C[2];
#endif

#if LIBINT2_DEFINED(eri, rho12_over_alpha1) || \
    LIBINT2_DEFINED(eri, rho12_over_alpha2)
    if (deriv_order_ > 0) {
#if LIBINT2_DEFINED(eri, rho12_over_alpha1)
      primdata.rho12_over_alpha1[0] = rhop_over_alpha1;
#endif
#if LIBINT2_DEFINED(eri, rho12_over_alpha2)
      primdata.rho12_over_alpha2[0] = alpha1 * oogammap;
#endif
    }
#endif
#if LIBINT2_DEFINED(eri, PC_x) && LIBINT2_DEFINED(eri, PC_y) && \
    LIBINT2_DEFINED(eri, PC_z)
    const auto PC2 = primdata.PC_x[0] * primdata.PC_x[0] +
                     primdata.PC_y[0] * primdata.PC_y[0] +
                     primdata.PC_z[0] * primdata.PC_z[0];
    const scalar_type U = gammap * PC2;
    const auto mmax = s1.contr[0].l + s2.contr[0].l + deriv_order_;
    auto *fm_ptr = &(primdata.LIBINT_T_S_ELECPOT_S(0)[0]);
    if (oper_ == Operator::nuclear) {
      const auto &fm_engine_ptr =
          any_cast<const detail::core_eval_pack_type<Operator::nuclear> &>(
              core_eval_pack_)
              .first();
      fm_engine_ptr->eval(fm_ptr, U, mmax);
    } else if (oper_ == Operator::erf_nuclear) {
      const auto &core_eval_ptr =
          any_cast<const detail::core_eval_pack_type<Operator::erf_nuclear> &>(
              core_eval_pack_)
              .first();
      const auto &core_ints_params = std::get<0>(
          any_cast<const typename operator_traits<
              Operator::erf_nuclear>::oper_params_type &>(core_ints_params_));
      core_eval_ptr->eval(fm_ptr, gammap, U, mmax, core_ints_params);
    } else if (oper_ == Operator::erfc_nuclear) {
      const auto &core_eval_ptr =
          any_cast<const detail::core_eval_pack_type<Operator::erfc_nuclear> &>(
              core_eval_pack_)
              .first();
      const auto &core_ints_params = std::get<0>(
          any_cast<const typename operator_traits<
              Operator::erfc_nuclear>::oper_params_type &>(core_ints_params_));
      core_eval_ptr->eval(fm_ptr, gammap, U, mmax, core_ints_params);
    }

    decltype(U) two_o_sqrt_PI(1.12837916709551257389615890312);
    const auto pfac =
        -q * sqrt(gammap) * two_o_sqrt_PI * ovlp_ss_x * ovlp_ss_y * ovlp_ss_z;
    const auto m_fence = mmax + 1;
    for (auto m = 0; m != m_fence; ++m) {
      fm_ptr[m] *= pfac;
    }
#endif
  }
}  // Engine::compute_primdata()

/// computes shell set of integrals of 2-body operator
/// \note result is stored in the "chemists"/Mulliken form, (tbra1 tbra2 |tket1
/// tket2), i.e. bra and ket are in chemists meaning; result is packed in
/// row-major order.
template <Operator op, BraKet bk, size_t der>
__libint2_engine_inline const Engine::target_ptr_vec &Engine::compute2(
    const libint2::Shell &tbra1, const libint2::Shell &tbra2,
    const libint2::Shell &tket1, const libint2::Shell &tket2,
    const ShellPair *tspbra, const ShellPair *tspket) {
  assert(op == oper_ && "Engine::compute2 -- operator mismatch");
  assert(bk == braket_ && "Engine::compute2 -- braket mismatch");
  assert(der == deriv_order_ && "Engine::compute2 -- deriv_order mismatch");
  assert(((tspbra == nullptr && tspket == nullptr) ||
          (tspbra != nullptr && tspket != nullptr)) &&
         "Engine::compute2 -- expects zero or two ShellPair objects");
  assert(screening_method_ != ScreeningMethod::Invalid);

  //
  // i.e. bra and ket refer to chemists bra and ket
  //

  // can only handle 1 contraction at a time
  assert((tbra1.ncontr() == 1 && tbra2.ncontr() == 1 && tket1.ncontr() == 1 &&
          tket2.ncontr() == 1) &&
         "generally-contracted shells are not yet supported");

  // angular momentum limit obeyed?
  assert(tbra1.contr[0].l <= lmax_ && "the angular momentum limit is exceeded");
  assert(tbra2.contr[0].l <= lmax_ && "the angular momentum limit is exceeded");
  assert(tket1.contr[0].l <= lmax_ && "the angular momentum limit is exceeded");
  assert(tket2.contr[0].l <= lmax_ && "the angular momentum limit is exceeded");

#if LIBINT2_SHELLQUARTET_SET == \
    LIBINT2_SHELLQUARTET_SET_STANDARD  // standard angular momentum ordering
  const auto swap_tbra = (tbra1.contr[0].l < tbra2.contr[0].l);
  const auto swap_tket = (tket1.contr[0].l < tket2.contr[0].l);
  const auto swap_braket =
      ((braket_ == BraKet::xx_xx) && (tbra1.contr[0].l + tbra2.contr[0].l >
                                      tket1.contr[0].l + tket2.contr[0].l)) ||
      braket_ == BraKet::xx_xs;
#else  // orca angular momentum ordering
  const auto swap_tbra = (tbra1.contr[0].l > tbra2.contr[0].l);
  const auto swap_tket = (tket1.contr[0].l > tket2.contr[0].l);
  const auto swap_braket =
      ((braket_ == BraKet::xx_xx) && (tbra1.contr[0].l + tbra2.contr[0].l <
                                      tket1.contr[0].l + tket2.contr[0].l)) ||
      braket_ == BraKet::xx_xs;
  assert(false && "feature not implemented");
  abort();
#endif
  const auto &bra1 =
      swap_braket ? (swap_tket ? tket2 : tket1) : (swap_tbra ? tbra2 : tbra1);
  const auto &bra2 =
      swap_braket ? (swap_tket ? tket1 : tket2) : (swap_tbra ? tbra1 : tbra2);
  const auto &ket1 =
      swap_braket ? (swap_tbra ? tbra2 : tbra1) : (swap_tket ? tket2 : tket1);
  const auto &ket2 =
      swap_braket ? (swap_tbra ? tbra1 : tbra2) : (swap_tket ? tket1 : tket2);
  const auto swap_bra = swap_braket ? swap_tket : swap_tbra;
  const auto swap_ket = swap_braket ? swap_tbra : swap_tket;
  // "permute" also the user-provided shell pair data
  const auto *spbra_precomputed = swap_braket ? tspket : tspbra;
  const auto *spket_precomputed = swap_braket ? tspbra : tspket;
  assert(((spbra_precomputed && spket_precomputed) ||
          (screening_method_ == ScreeningMethod::Original ||
           screening_method_ == ScreeningMethod::Conservative)) &&
         "Engine::compute2: without precomputed shell pair data can only use "
         "original or conservative screening methods");

  const auto tform = bra1.contr[0].pure || bra2.contr[0].pure ||
                     ket1.contr[0].pure || ket2.contr[0].pure;
  const auto permute = swap_braket || swap_tbra || swap_tket;
  const auto use_scratch = permute || tform;

  // assert # of primitive pairs
  auto nprim_bra1 = bra1.nprim();
  auto nprim_bra2 = bra2.nprim();
  auto nprim_ket1 = ket1.nprim();
  auto nprim_ket2 = ket2.nprim();

  // adjust max angular momentum, if needed
  auto lmax = std::max(std::max(bra1.contr[0].l, bra2.contr[0].l),
                       std::max(ket1.contr[0].l, ket2.contr[0].l));
  assert(lmax <= lmax_ && "the angular momentum limit is exceeded");
  const auto lmax_bra = std::max(bra1.contr[0].l, bra2.contr[0].l);
  const auto lmax_ket = std::max(ket1.contr[0].l, ket2.contr[0].l);

#ifdef LIBINT2_ENGINE_PROFILE_CLASS
  class_id id(bra1.contr[0].l, bra2.contr[0].l, ket1.contr[0].l,
              ket2.contr[0].l);
  if (class_profiles.find(id) == class_profiles.end()) {
    class_profile dummy;
    class_profiles[id] = dummy;
  }
#endif

// compute primitive data
#ifdef LIBINT2_ENGINE_TIMERS
  timers.start(0);
#endif
  {
    auto p = 0;
    // initialize shell pairs, if not given ...
    // since screening primitive pairs for bra is not possible without knowing
    // the worst-case primitive data in ket (and vice versa) using ln_precision_
    // is not safe, but should work fine for moderate basis sets ... precompute
    // shell pair data yourself to guarantee proper screening of primitive pairs
    // (see Engine::set_precision() for details of how to screen primitives for
    // a given target precision of the integrals)
    const auto target_shellpair_ln_precision = ln_precision_;
    const auto recompute_spbra =
        !spbra_precomputed ||
        spbra_precomputed->ln_prec > target_shellpair_ln_precision;
    const auto recompute_spket =
        !spket_precomputed ||
        spket_precomputed->ln_prec > target_shellpair_ln_precision;
    const ShellPair &spbra =
        recompute_spbra
            ? (spbra_.init(bra1, bra2, target_shellpair_ln_precision,
                           screening_method_),
               spbra_)
            : *spbra_precomputed;
    const ShellPair &spket =
        recompute_spket
            ? (spket_.init(ket1, ket2, target_shellpair_ln_precision,
                           screening_method_),
               spket_)
            : *spket_precomputed;
    assert(spbra.screening_method_ == screening_method_ &&
           spket.screening_method_ == screening_method_ &&
           "Engine::compute2: received ShellPair initialized for an "
           "incompatible screening method");
    // determine whether shell pair data refers to the actual ({bra1,bra2}) or
    // swapped ({bra2,bra1}) pairs if computed the shell pair data here then
    // it's always in actual order, otherwise check swap_bra/swap_ket
    const auto spbra_is_swapped = recompute_spbra ? false : swap_bra;
    const auto spket_is_swapped = recompute_spket ? false : swap_ket;

    using real_t = Shell::real_t;
    // swapping bra turns AB into BA = -AB
    real_t BA[3];
    if (spbra_is_swapped) {
      for (auto xyz = 0; xyz != 3; ++xyz) BA[xyz] = -spbra_precomputed->AB[xyz];
    }
    const auto &AB = spbra_is_swapped ? BA : spbra.AB;
    // swapping ket turns CD into DC = -CD
    real_t DC[3];
    if (spket_is_swapped) {
      for (auto xyz = 0; xyz != 3; ++xyz) DC[xyz] = -spket_precomputed->AB[xyz];
    }
    const auto &CD = spket_is_swapped ? DC : spket.AB;

    const auto &A = bra1.O;
    const auto &B = bra2.O;
    const auto &C = ket1.O;
    const auto &D = ket2.O;

    // compute all primitive quartet data
    const auto npbra = spbra.primpairs.size();
    const auto npket = spket.primpairs.size();
    const scalar_type npbraket = npbra * npket;
    for (auto pb = 0; pb != npbra; ++pb) {
      for (auto pk = 0; pk != npket; ++pk) {
        // primitive quartet coarse screening:
        if (spbra.primpairs[pb].ln_scr + spket.primpairs[pk].ln_scr >
            ln_precision_) {
          Libint_t &primdata = primdata_[p];
          const auto &sbra1 = bra1;
          const auto &sbra2 = bra2;
          const auto &sket1 = ket1;
          const auto &sket2 = ket2;
          auto pbra = pb;
          auto pket = pk;

          const auto &spbrapp = spbra.primpairs[pbra];
          const auto &spketpp = spket.primpairs[pket];
          // if shell-pair data given by user
          const auto &pbra1 = spbra_is_swapped ? spbrapp.p2 : spbrapp.p1;
          const auto &pbra2 = spbra_is_swapped ? spbrapp.p1 : spbrapp.p2;
          const auto &pket1 = spket_is_swapped ? spketpp.p2 : spketpp.p1;
          const auto &pket2 = spket_is_swapped ? spketpp.p1 : spketpp.p2;

          const auto alpha0 = sbra1.alpha[pbra1];
          const auto alpha1 = sbra2.alpha[pbra2];
          const auto alpha2 = sket1.alpha[pket1];
          const auto alpha3 = sket2.alpha[pket2];

          const auto c0 = sbra1.contr[0].coeff[pbra1];
          const auto c1 = sbra2.contr[0].coeff[pbra2];
          const auto c2 = sket1.contr[0].coeff[pket1];
          const auto c3 = sket2.contr[0].coeff[pket2];

          const auto l0 = sbra1.contr[0].l;
          const auto l1 = sbra2.contr[0].l;
          const auto l2 = sket1.contr[0].l;
          const auto l3 = sket2.contr[0].l;
          const auto l = l0 + l1 + l2 + l3;

          const auto gammap = alpha0 + alpha1;
          const auto oogammap = spbrapp.one_over_gamma;
          const auto rhop = alpha0 * alpha1 * oogammap;

          const auto gammaq = alpha2 + alpha3;
          const auto oogammaq = spketpp.one_over_gamma;
          const auto rhoq = alpha2 * alpha3 * oogammaq;

          const auto &P = spbrapp.P;
          const auto &Q = spketpp.P;
          const auto PQx = P[0] - Q[0];
          const auto PQy = P[1] - Q[1];
          const auto PQz = P[2] - Q[2];
          const auto PQ2 = PQx * PQx + PQy * PQy + PQz * PQz;

          const auto K12 = spbrapp.K * spketpp.K;
          const auto gammapq = gammap + gammaq;
          const auto sqrt_gammapq = sqrt(gammapq);
          const auto oogammapq = 1.0 / (gammapq);
          auto pfac = K12 * sqrt_gammapq * oogammapq;
          pfac *= c0 * c1 * c2 * c3 * scale_;

          // original and conservative methods: screen primitive integral using
          // actual pfac
          if (static_cast<int>(screening_method_) &
              (static_cast<int>(ScreeningMethod::Original) |
               static_cast<int>(ScreeningMethod::Conservative))) {
            scalar_type magnitude_estimate = std::abs(pfac);
            if (screening_method_ == ScreeningMethod::Conservative) {
              // magnitude of primitive (ab|cd) integral for nonzero L differs
              // from that of (00|00) geometric and exponent-dependent factor,
              // some of which only depend on bra or ket, and some are bra-ket
              // dependent ... here we account only for bra-only and ket-only
              // nonspherical factors
              const auto nonspherical_pfac_magnitude = std::max(
                  1., spbrapp.nonsph_screen_fac * spketpp.nonsph_screen_fac);
              magnitude_estimate *= nonspherical_pfac_magnitude * npbraket;
            }
            if (magnitude_estimate < precision_) continue;
          }

          {
            const scalar_type rho = gammap * gammaq * oogammapq;
            const scalar_type T = PQ2 * rho;
            auto *gm_ptr = &(primdata.LIBINT_T_SS_EREP_SS(0)[0]);
            const auto mmax = l + deriv_order_;

            if (!skip_core_ints) {
              switch (oper_) {
                case Operator::coulomb: {
                  const auto &core_eval_ptr =
                      any_cast<const detail::core_eval_pack_type<
                          Operator::coulomb> &>(core_eval_pack_)
                          .first();
                  core_eval_ptr->eval(gm_ptr, T, mmax);
                } break;
                case Operator::cgtg_x_coulomb: {
                  const auto &core_eval_ptr =
                      any_cast<const detail::core_eval_pack_type<
                          Operator::cgtg_x_coulomb> &>(core_eval_pack_)
                          .first();
                  auto &core_eval_scratch =
                      any_cast<detail::core_eval_pack_type<
                          Operator::cgtg_x_coulomb> &>(core_eval_pack_)
                          .second();
                  const auto &core_ints_params =
                      any_cast<const typename operator_traits<
                          Operator::cgtg>::oper_params_type &>(
                          core_ints_params_);
                  core_eval_ptr->eval(gm_ptr, rho, T, mmax, core_ints_params,
                                      &core_eval_scratch);
                } break;
                case Operator::cgtg: {
                  const auto &core_eval_ptr =
                      any_cast<
                          const detail::core_eval_pack_type<Operator::cgtg> &>(
                          core_eval_pack_)
                          .first();
                  const auto &core_ints_params =
                      any_cast<const typename operator_traits<
                          Operator::cgtg>::oper_params_type &>(
                          core_ints_params_);
                  core_eval_ptr->eval(gm_ptr, rho, T, mmax, core_ints_params);
                } break;
                case Operator::delcgtg2: {
                  const auto &core_eval_ptr =
                      any_cast<const detail::core_eval_pack_type<
                          Operator::delcgtg2> &>(core_eval_pack_)
                          .first();
                  const auto &core_ints_params =
                      any_cast<const typename operator_traits<
                          Operator::cgtg>::oper_params_type &>(
                          core_ints_params_);
                  core_eval_ptr->eval(gm_ptr, rho, T, mmax, core_ints_params);
                } break;
                case Operator::delta: {
                  const auto &core_eval_ptr =
                      any_cast<
                          const detail::core_eval_pack_type<Operator::delta> &>(
                          core_eval_pack_)
                          .first();
                  core_eval_ptr->eval(gm_ptr, rho, T, mmax);
                } break;
                case Operator::r12: {
                  const auto &core_eval_ptr =
                      any_cast<
                          const detail::core_eval_pack_type<Operator::r12> &>(
                          core_eval_pack_)
                          .first();
                  core_eval_ptr->eval(gm_ptr, rho, T, mmax);
                } break;
                case Operator::erf_coulomb: {
                  const auto &core_eval_ptr =
                      any_cast<const detail::core_eval_pack_type<
                          Operator::erf_coulomb> &>(core_eval_pack_)
                          .first();
                  const auto &core_ints_params =
                      any_cast<const typename operator_traits<
                          Operator::erf_coulomb>::oper_params_type &>(
                          core_ints_params_);
                  core_eval_ptr->eval(gm_ptr, rho, T, mmax, core_ints_params);
                } break;
                case Operator::erfc_coulomb: {
                  const auto &core_eval_ptr =
                      any_cast<const detail::core_eval_pack_type<
                          Operator::erfc_coulomb> &>(core_eval_pack_)
                          .first();
                  const auto &core_ints_params =
                      any_cast<const typename operator_traits<
                          Operator::erfc_coulomb>::oper_params_type &>(
                          core_ints_params_);
                  core_eval_ptr->eval(gm_ptr, rho, T, mmax, core_ints_params);
                } break;
                case Operator::stg: {
                  const auto &core_eval_ptr =
                      any_cast<
                          const detail::core_eval_pack_type<Operator::stg> &>(
                          core_eval_pack_)
                          .first();
                  auto zeta = any_cast<const typename operator_traits<
                      Operator::stg>::oper_params_type &>(core_ints_params_);
                  const auto one_over_rho = gammapq * oogammap * oogammaq;
                  core_eval_ptr->eval_slater(gm_ptr, one_over_rho, T, mmax,
                                             zeta);
                } break;
                case Operator::stg_x_coulomb: {
                  const auto &core_eval_ptr =
                      any_cast<
                          const detail::core_eval_pack_type<Operator::stg> &>(
                          core_eval_pack_)
                          .first();
                  auto zeta = any_cast<const typename operator_traits<
                      Operator::stg>::oper_params_type &>(core_ints_params_);
                  const auto one_over_rho = gammapq * oogammap * oogammaq;
                  core_eval_ptr->eval_yukawa(gm_ptr, one_over_rho, T, mmax,
                                             zeta);
                } break;
                default:
                  assert(false && "missing case in a switch");  // unreachable
                  abort();
              }
            }

            for (auto m = 0; m != mmax + 1; ++m) {
              gm_ptr[m] *= pfac;
            }

            if (mmax != 0) {
              if (braket_ == BraKet::xx_xx) {
#if LIBINT2_DEFINED(eri, PA_x)
                primdata.PA_x[0] = P[0] - A[0];
#endif
#if LIBINT2_DEFINED(eri, PA_y)
                primdata.PA_y[0] = P[1] - A[1];
#endif
#if LIBINT2_DEFINED(eri, PA_z)
                primdata.PA_z[0] = P[2] - A[2];
#endif
#if LIBINT2_DEFINED(eri, PB_x)
                primdata.PB_x[0] = P[0] - B[0];
#endif
#if LIBINT2_DEFINED(eri, PB_y)
                primdata.PB_y[0] = P[1] - B[1];
#endif
#if LIBINT2_DEFINED(eri, PB_z)
                primdata.PB_z[0] = P[2] - B[2];
#endif
              }

              if (braket_ != BraKet::xs_xs) {
#if LIBINT2_DEFINED(eri, QC_x)
                primdata.QC_x[0] = Q[0] - C[0];
#endif
#if LIBINT2_DEFINED(eri, QC_y)
                primdata.QC_y[0] = Q[1] - C[1];
#endif
#if LIBINT2_DEFINED(eri, QC_z)
                primdata.QC_z[0] = Q[2] - C[2];
#endif
#if LIBINT2_DEFINED(eri, QD_x)
                primdata.QD_x[0] = Q[0] - D[0];
#endif
#if LIBINT2_DEFINED(eri, QD_y)
                primdata.QD_y[0] = Q[1] - D[1];
#endif
#if LIBINT2_DEFINED(eri, QD_z)
                primdata.QD_z[0] = Q[2] - D[2];
#endif
              }

              if (braket_ == BraKet::xx_xx) {
#if LIBINT2_DEFINED(eri, AB_x)
                primdata.AB_x[0] = AB[0];
#endif
#if LIBINT2_DEFINED(eri, AB_y)
                primdata.AB_y[0] = AB[1];
#endif
#if LIBINT2_DEFINED(eri, AB_z)
                primdata.AB_z[0] = AB[2];
#endif
#if LIBINT2_DEFINED(eri, BA_x)
                primdata.BA_x[0] = -AB[0];
#endif
#if LIBINT2_DEFINED(eri, BA_y)
                primdata.BA_y[0] = -AB[1];
#endif
#if LIBINT2_DEFINED(eri, BA_z)
                primdata.BA_z[0] = -AB[2];
#endif
              }

              if (braket_ != BraKet::xs_xs) {
#if LIBINT2_DEFINED(eri, CD_x)
                primdata.CD_x[0] = CD[0];
#endif
#if LIBINT2_DEFINED(eri, CD_y)
                primdata.CD_y[0] = CD[1];
#endif
#if LIBINT2_DEFINED(eri, CD_z)
                primdata.CD_z[0] = CD[2];
#endif
#if LIBINT2_DEFINED(eri, DC_x)
                primdata.DC_x[0] = -CD[0];
#endif
#if LIBINT2_DEFINED(eri, DC_y)
                primdata.DC_y[0] = -CD[1];
#endif
#if LIBINT2_DEFINED(eri, DC_z)
                primdata.DC_z[0] = -CD[2];
#endif
              }

              const auto gammap_o_gammapgammaq = oogammapq * gammap;
              const auto gammaq_o_gammapgammaq = oogammapq * gammaq;

              const auto Wx =
                  (gammap_o_gammapgammaq * P[0] + gammaq_o_gammapgammaq * Q[0]);
              const auto Wy =
                  (gammap_o_gammapgammaq * P[1] + gammaq_o_gammapgammaq * Q[1]);
              const auto Wz =
                  (gammap_o_gammapgammaq * P[2] + gammaq_o_gammapgammaq * Q[2]);

              if (deriv_order_ > 0 || lmax_bra > 0) {
#if LIBINT2_DEFINED(eri, WP_x)
                primdata.WP_x[0] = Wx - P[0];
#endif
#if LIBINT2_DEFINED(eri, WP_y)
                primdata.WP_y[0] = Wy - P[1];
#endif
#if LIBINT2_DEFINED(eri, WP_z)
                primdata.WP_z[0] = Wz - P[2];
#endif
              }
              if (deriv_order_ > 0 || lmax_ket > 0) {
#if LIBINT2_DEFINED(eri, WQ_x)
                primdata.WQ_x[0] = Wx - Q[0];
#endif
#if LIBINT2_DEFINED(eri, WQ_y)
                primdata.WQ_y[0] = Wy - Q[1];
#endif
#if LIBINT2_DEFINED(eri, WQ_z)
                primdata.WQ_z[0] = Wz - Q[2];
#endif
              }
#if LIBINT2_DEFINED(eri, oo2z)
              primdata.oo2z[0] = 0.5 * oogammap;
#endif
#if LIBINT2_DEFINED(eri, oo2e)
              primdata.oo2e[0] = 0.5 * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, oo2ze)
              primdata.oo2ze[0] = 0.5 * oogammapq;
#endif
#if LIBINT2_DEFINED(eri, roz)
              primdata.roz[0] = rho * oogammap;
#endif
#if LIBINT2_DEFINED(eri, roe)
              primdata.roe[0] = rho * oogammaq;
#endif

// using ITR?
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_0_0_x)
              primdata.TwoPRepITR_pfac0_0_0_x[0] =
                  -(alpha1 * AB[0] + alpha3 * CD[0]) * oogammap;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_0_0_y)
              primdata.TwoPRepITR_pfac0_0_0_y[0] =
                  -(alpha1 * AB[1] + alpha3 * CD[1]) * oogammap;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_0_0_z)
              primdata.TwoPRepITR_pfac0_0_0_z[0] =
                  -(alpha1 * AB[2] + alpha3 * CD[2]) * oogammap;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_1_0_x)
              primdata.TwoPRepITR_pfac0_1_0_x[0] =
                  -(alpha1 * AB[0] + alpha3 * CD[0]) * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_1_0_y)
              primdata.TwoPRepITR_pfac0_1_0_y[0] =
                  -(alpha1 * AB[1] + alpha3 * CD[1]) * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_1_0_z)
              primdata.TwoPRepITR_pfac0_1_0_z[0] =
                  -(alpha1 * AB[2] + alpha3 * CD[2]) * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_0_1_x)
              primdata.TwoPRepITR_pfac0_0_1_x[0] =
                  (alpha0 * AB[0] + alpha2 * CD[0]) * oogammap;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_0_1_y)
              primdata.TwoPRepITR_pfac0_0_1_y[0] =
                  (alpha0 * AB[1] + alpha2 * CD[1]) * oogammap;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_0_1_z)
              primdata.TwoPRepITR_pfac0_0_1_z[0] =
                  (alpha0 * AB[2] + alpha2 * CD[2]) * oogammap;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_1_1_x)
              primdata.TwoPRepITR_pfac0_1_1_x[0] =
                  (alpha0 * AB[0] + alpha2 * CD[0]) * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_1_1_y)
              primdata.TwoPRepITR_pfac0_1_1_y[0] =
                  (alpha0 * AB[1] + alpha2 * CD[1]) * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, TwoPRepITR_pfac0_1_1_z)
              primdata.TwoPRepITR_pfac0_1_1_z[0] =
                  (alpha0 * AB[2] + alpha2 * CD[2]) * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, eoz)
              primdata.eoz[0] = gammaq * oogammap;
#endif
#if LIBINT2_DEFINED(eri, zoe)
              primdata.zoe[0] = gammap * oogammaq;
#endif

              // prefactors for derivative ERI relations
              if (deriv_order_ > 0) {
#if LIBINT2_DEFINED(eri, alpha1_rho_over_zeta2)
                primdata.alpha1_rho_over_zeta2[0] =
                    alpha0 * (oogammap * gammaq_o_gammapgammaq);
#endif
#if LIBINT2_DEFINED(eri, alpha2_rho_over_zeta2)
                primdata.alpha2_rho_over_zeta2[0] =
                    alpha1 * (oogammap * gammaq_o_gammapgammaq);
#endif
#if LIBINT2_DEFINED(eri, alpha3_rho_over_eta2)
                primdata.alpha3_rho_over_eta2[0] =
                    alpha2 * (oogammaq * gammap_o_gammapgammaq);
#endif
#if LIBINT2_DEFINED(eri, alpha4_rho_over_eta2)
                primdata.alpha4_rho_over_eta2[0] =
                    alpha3 * (oogammaq * gammap_o_gammapgammaq);
#endif
#if LIBINT2_DEFINED(eri, alpha1_over_zetapluseta)
                primdata.alpha1_over_zetapluseta[0] = alpha0 * oogammapq;
#endif
#if LIBINT2_DEFINED(eri, alpha2_over_zetapluseta)
                primdata.alpha2_over_zetapluseta[0] = alpha1 * oogammapq;
#endif
#if LIBINT2_DEFINED(eri, alpha3_over_zetapluseta)
                primdata.alpha3_over_zetapluseta[0] = alpha2 * oogammapq;
#endif
#if LIBINT2_DEFINED(eri, alpha4_over_zetapluseta)
                primdata.alpha4_over_zetapluseta[0] = alpha3 * oogammapq;
#endif
#if LIBINT2_DEFINED(eri, rho12_over_alpha1)
                primdata.rho12_over_alpha1[0] = alpha1 * oogammap;
#endif
#if LIBINT2_DEFINED(eri, rho12_over_alpha2)
                primdata.rho12_over_alpha2[0] = alpha0 * oogammap;
#endif
#if LIBINT2_DEFINED(eri, rho34_over_alpha3)
                primdata.rho34_over_alpha3[0] = alpha3 * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, rho34_over_alpha4)
                primdata.rho34_over_alpha4[0] = alpha2 * oogammaq;
#endif
#if LIBINT2_DEFINED(eri, two_alpha0_bra)
                primdata.two_alpha0_bra[0] = 2.0 * alpha0;
#endif
#if LIBINT2_DEFINED(eri, two_alpha0_ket)
                primdata.two_alpha0_ket[0] = 2.0 * alpha1;
#endif
#if LIBINT2_DEFINED(eri, two_alpha1_bra)
                primdata.two_alpha1_bra[0] = 2.0 * alpha2;
#endif
#if LIBINT2_DEFINED(eri, two_alpha1_ket)
                primdata.two_alpha1_ket[0] = 2.0 * alpha3;
#endif
              }
            }  // m != 0

            ++p;
          }  // prefac-based prim quartet screen

        }  // rough prim quartet screen based on pair values
      }    // ket prim pair
    }      // bra prim pair
    primdata_[0].contrdepth = p;
  }

#ifdef LIBINT2_ENGINE_TIMERS
  const auto t0 = timers.stop(0);
#ifdef LIBINT2_ENGINE_PROFILE_CLASS
  class_profiles[id].prereqs += t0.count();
  if (primdata_[0].contrdepth != 0) {
    class_profiles[id].nshellset += 1;
    class_profiles[id].nprimset += primdata_[0].contrdepth;
  }
#endif
#endif

  // all primitive combinations screened out? set 1st target ptr to nullptr
  if (primdata_[0].contrdepth == 0) {
    targets_[0] = nullptr;
    return targets_;
  }

  // compute directly (ss|ss)
  const auto compute_directly = lmax == 0 && deriv_order_ == 0;

  if (compute_directly) {
#ifdef LIBINT2_ENGINE_TIMERS
    timers.start(1);
#endif
    auto &stack = primdata_[0].stack[0];
    stack = 0;
    for (auto p = 0; p != primdata_[0].contrdepth; ++p)
      stack += primdata_[p].LIBINT_T_SS_EREP_SS(0)[0];
    primdata_[0].targets[0] = primdata_[0].stack;
#ifdef LIBINT2_ENGINE_TIMERS
    const auto t1 = timers.stop(1);
#ifdef LIBINT2_ENGINE_PROFILE_CLASS
    class_profiles[id].build_vrr += t1.count();
#endif
#endif
  }       // compute directly
  else {  // call libint
#ifdef LIBINT2_ENGINE_TIMERS
#ifdef LIBINT2_PROFILE
    const auto t1_hrr_start = primdata_[0].timers->read(0);
    const auto t1_vrr_start = primdata_[0].timers->read(1);
#endif
    timers.start(1);
#endif

    size_t buildfnidx;
    switch (braket_) {
      case BraKet::xx_xx:
        buildfnidx =
            ((bra1.contr[0].l * hard_lmax_ + bra2.contr[0].l) * hard_lmax_ +
             ket1.contr[0].l) *
                hard_lmax_ +
            ket2.contr[0].l;
        break;

      case BraKet::xx_xs:
        assert(false && "this braket is not supported");
        abort();
        break;
      case BraKet::xs_xx: {
        /// lmax might be center dependent
        int ket_lmax = hard_lmax_;
        switch (deriv_order_) {
#define BOOST_PP_NBODYENGINE_MCR8(r, data, i, elem)                      \
  case i:                                                                \
    BOOST_PP_IF(                                                         \
        BOOST_PP_IS_1(BOOST_PP_CAT(                                      \
            LIBINT2_CENTER_DEPENDENT_MAX_AM_3eri,                        \
            BOOST_PP_IIF(BOOST_PP_GREATER(i, 0), i, BOOST_PP_EMPTY()))), \
        ket_lmax = hard_default_lmax_, BOOST_PP_EMPTY());                \
    break;

          BOOST_PP_LIST_FOR_EACH_I(BOOST_PP_NBODYENGINE_MCR8, _,
                                   BOOST_PP_NBODY_DERIV_ORDER_LIST)

          default:
            assert(false && "missing case in switch");
            abort();
        }
        buildfnidx = (bra1.contr[0].l * ket_lmax + ket1.contr[0].l) * ket_lmax +
                     ket2.contr[0].l;
#ifdef ERI3_PURE_SH
        if (bra1.contr[0].l > 1)
          assert(bra1.contr[0].pure &&
                 "library assumes a solid harmonics shell in bra of a 3-center "
                 "2-body int, but a cartesian shell given");
#endif
      } break;

      case BraKet::xs_xs:
        buildfnidx = bra1.contr[0].l * hard_lmax_ + ket1.contr[0].l;
#ifdef ERI2_PURE_SH
        if (bra1.contr[0].l > 1)
          assert(bra1.contr[0].pure &&
                 "library assumes solid harmonics shells in a 2-center "
                 "2-body int, but a cartesian shell given in bra");
        if (ket1.contr[0].l > 1)
          assert(ket1.contr[0].pure &&
                 "library assumes solid harmonics shells in a 2-center "
                 "2-body int, but a cartesian shell given in bra");
#endif
        break;

      default:
        assert(false && "invalid braket");
        abort();
    }

    assert(buildfnptrs_[buildfnidx] && "null build function ptr");
    buildfnptrs_[buildfnidx](&primdata_[0]);

#ifdef LIBINT2_ENGINE_TIMERS
    const auto t1 = timers.stop(1);
#ifdef LIBINT2_ENGINE_PROFILE_CLASS
#ifndef LIBINT2_PROFILE
    class_profiles[id].build_vrr += t1.count();
#else
    class_profiles[id].build_hrr += primdata_[0].timers->read(0) - t1_hrr_start;
    class_profiles[id].build_vrr += primdata_[0].timers->read(1) - t1_vrr_start;
#endif
#endif
#endif

#ifdef LIBINT2_ENGINE_TIMERS
    timers.start(2);
#endif

    const auto ntargets = nshellsets();

    // if needed, permute and transform
    if (use_scratch) {
      constexpr auto using_scalar_real =
          sizeof(value_type) == sizeof(scalar_type);
      static_assert(using_scalar_real,
                    "Libint2 C++11 API only supports scalar real types");
      typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic,
                            Eigen::RowMajor>
          Matrix;

      // a 2-d view of the 4-d source tensor
      const auto nr1_cart = bra1.cartesian_size();
      const auto nr2_cart = bra2.cartesian_size();
      const auto nc1_cart = ket1.cartesian_size();
      const auto nc2_cart = ket2.cartesian_size();
      const auto ncol_cart = nc1_cart * nc2_cart;
      const auto n1234_cart = nr1_cart * nr2_cart * ncol_cart;
      const auto nr1 = bra1.size();
      const auto nr2 = bra2.size();
      const auto nc1 = ket1.size();
      const auto nc2 = ket2.size();
      const auto nrow = nr1 * nr2;
      const auto ncol = nc1 * nc2;

      // a 2-d view of the 4-d target tensor
      const auto nr1_tgt = tbra1.size();
      const auto nr2_tgt = tbra2.size();
      const auto nc1_tgt = tket1.size();
      const auto nc2_tgt = tket2.size();
      const auto ncol_tgt = nc1_tgt * nc2_tgt;
      const auto n_tgt = nr1_tgt * nr2_tgt * ncol_tgt;

      auto hotscr = &scratch_[0];  // points to the hot scratch

      // transform to solid harmonics first, then unpermute, if necessary
      for (auto s = 0; s != ntargets; ++s) {
        // when permuting derivatives may need to permute shellsets also, not
        // just integrals
        // within shellsets; this will point to where source shellset s should
        // end up
        auto s_target = s;

        auto source =
            primdata_[0].targets[s];  // points to the most recent result
        auto target = hotscr;

        if (bra1.contr[0].pure) {
          libint2::solidharmonics::transform_first(
              bra1.contr[0].l, nr2_cart * ncol_cart, source, target);
          std::swap(source, target);
        }
        if (bra2.contr[0].pure) {
          libint2::solidharmonics::transform_inner(bra1.size(), bra2.contr[0].l,
                                                   ncol_cart, source, target);
          std::swap(source, target);
        }
        if (ket1.contr[0].pure) {
          libint2::solidharmonics::transform_inner(nrow, ket1.contr[0].l,
                                                   nc2_cart, source, target);
          std::swap(source, target);
        }
        if (ket2.contr[0].pure) {
          libint2::solidharmonics::transform_last(
              bra1.size() * bra2.size() * ket1.size(), ket2.contr[0].l, source,
              target);
          std::swap(source, target);
        }

        // need to permute?
        if (permute) {
          // loop over rows of the source matrix
          const auto *src_row_ptr = source;
          auto tgt_ptr = target;

          // if permuting derivatives ints must update their derivative index
          // Additional BraKet types would require adding support to
          // DerivMapGenerator::generate_deriv_index_map
          if (deriv_order_) {
            Tensor<size_t> &mapDerivIndex =
                libint2::DerivMapGenerator::instance(deriv_order_, braket_);
            switch (braket_) {
              case BraKet::xx_xx: {
                s_target = mapDerivIndex((size_t)swap_braket, (size_t)swap_tbra,
                                         (size_t)swap_tket, (size_t)s);
              } break;
              case BraKet::xs_xx: {
                assert(!swap_bra);
                assert(!swap_braket);
                s_target = mapDerivIndex((size_t)0, (size_t)0,
                                         (size_t)swap_tket, (size_t)s);
              } break;
              case BraKet::xs_xs: {
                assert(!swap_bra);
                assert(!swap_ket);
                assert(!swap_braket);
                s_target = s;
              } break;

              default:
                assert(false &&
                       "This braket type not yet supported for geometric "
                       "derivatives");
                abort();
            }
          }

          for (auto r1 = 0; r1 != nr1; ++r1) {
            for (auto r2 = 0; r2 != nr2; ++r2, src_row_ptr += ncol) {
              typedef Eigen::Map<const Matrix> ConstMap;
              typedef Eigen::Map<Matrix> Map;
              typedef Eigen::Map<Matrix, Eigen::Unaligned,
                                 Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
                  StridedMap;

              // represent this source row as a matrix
              ConstMap src_blk_mat(src_row_ptr, nc1, nc2);

              // and copy to the block of the target matrix
              if (swap_braket) {
                // if swapped bra and ket, a row of source becomes a column
                // of
                // target
                // source row {r1,r2} is mapped to target column {r1,r2} if
                // !swap_tket, else to {r2,r1}
                const auto tgt_col_idx =
                    !swap_tket ? r1 * nr2 + r2 : r2 * nr1 + r1;
                StridedMap tgt_blk_mat(
                    tgt_ptr + tgt_col_idx, nr1_tgt, nr2_tgt,
                    Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(
                        nr2_tgt * ncol_tgt, ncol_tgt));
                if (swap_tbra)
                  tgt_blk_mat = src_blk_mat.transpose();
                else
                  tgt_blk_mat = src_blk_mat;
              } else {
                // source row {r1,r2} is mapped to target row {r1,r2} if
                // !swap_tbra, else to {r2,r1}
                const auto tgt_row_idx =
                    !swap_tbra ? r1 * nr2 + r2 : r2 * nr1 + r1;
                Map tgt_blk_mat(tgt_ptr + tgt_row_idx * ncol, nc1_tgt, nc2_tgt);
                if (swap_tket)
                  tgt_blk_mat = src_blk_mat.transpose();
                else
                  tgt_blk_mat = src_blk_mat;
              }
            }  // end of loop
          }    // over rows of source
          std::swap(source, target);
        }  // need to permute?

        // if the integrals ended up in scratch_, keep them there, update the
        // hot buffer
        // to the next available scratch space, and update targets_
        if (source != primdata_[0].targets[s]) {
          hotscr += n1234_cart;
          if (s != s_target)
            assert(set_targets_ && "logic error");  // mess if targets_ points
                                                    // to primdata_[0].targets
          targets_[s_target] = source;
        } else {
          // only needed if permuted derivs or set_targets_ is true
          // for simplicity always set targets_
          if (s != s_target)
            assert(set_targets_ && "logic error");  // mess if targets_ points
                                                    // to primdata_[0].targets
          targets_[s_target] = source;
        }
      }     // loop over shellsets
    }       // if need_scratch => needed to transpose and/or tform
    else {  // did not use scratch? may still need to update targets_
      if (set_targets_) {
        for (auto s = 0; s != ntargets; ++s)
          targets_[s] = primdata_[0].targets[s];
      }
    }

#ifdef LIBINT2_ENGINE_TIMERS
    const auto t2 = timers.stop(2);
#ifdef LIBINT2_ENGINE_PROFILE_CLASS
    class_profiles[id].tform += t2.count();
#endif
#endif
  }  // not (ss|ss)

  if (cartesian_shell_normalization() == CartesianShellNormalization::uniform) {
    std::array<std::reference_wrapper<const Shell>, 4> shells{tbra1, tbra2,
                                                              tket1, tket2};
    for (auto s = 0ul; s != targets_.size(); ++s) {
      uniform_normalize_cartesian_shells(const_cast<value_type *>(targets_[s]),
                                         shells);
    }
  }

  return targets_;
}

#undef BOOST_PP_NBODY_OPERATOR_LIST
#undef BOOST_PP_NBODY_OPERATOR_INDEX_TUPLE
#undef BOOST_PP_NBODY_OPERATOR_INDEX_LIST
#undef BOOST_PP_NBODY_BRAKET_INDEX_TUPLE
#undef BOOST_PP_NBODY_BRAKET_INDEX_LIST
#undef BOOST_PP_NBODY_DERIV_ORDER_TUPLE
#undef BOOST_PP_NBODY_DERIV_ORDER_LIST
#undef BOOST_PP_NBODYENGINE_MCR3
#undef BOOST_PP_NBODYENGINE_MCR3_ncenter
#undef BOOST_PP_NBODYENGINE_MCR3_default_ncenter
#undef BOOST_PP_NBODYENGINE_MCR3_NCENTER
#undef BOOST_PP_NBODYENGINE_MCR3_OPER
#undef BOOST_PP_NBODYENGINE_MCR3_DERIV
#undef BOOST_PP_NBODYENGINE_MCR3_task
#undef BOOST_PP_NBODYENGINE_MCR3_TASK
#undef BOOST_PP_NBODYENGINE_MCR4
#undef BOOST_PP_NBODYENGINE_MCR5
#undef BOOST_PP_NBODYENGINE_MCR6
#undef BOOST_PP_NBODYENGINE_MCR7

#ifdef LIBINT2_DOES_NOT_INLINE_ENGINE
template any Engine::enforce_params_type<Engine::empty_pod>(
    Operator oper, const Engine::empty_pod &params, bool throw_if_wrong_type);

template const Engine::target_ptr_vec &Engine::compute<Shell>(
    const Shell &first_shell, const Shell &);

template const Engine::target_ptr_vec &Engine::compute<Shell, Shell>(
    const Shell &first_shell, const Shell &, const Shell &);

template const Engine::target_ptr_vec &Engine::compute<Shell, Shell, Shell>(
    const Shell &first_shell, const Shell &, const Shell &, const Shell &);
#endif

}  // namespace libint2

#endif /* _libint2_src_lib_libint_engineimpl_h_ */

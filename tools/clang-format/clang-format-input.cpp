// concat of the source examples from the CLion C/C++ Code Style settings

// Tabs and Indents

#if !defined(OS)
#define OS_NOT_DEFINED
#endif
/*********************************************
 * ...globalFunc...
 *********************************************/
void globalFunc();
namespace foo
{
  /**
  ...Foo...
  */
  class Foo
  {
  public:
    Foo();
    ~Foo();
    virtual Foo *getSelf() { return Foo::getSelf(); }

  private:
    void innerFunc();
    int var;
  };
} // namespace foo
struct FooPOD
{
#ifdef OS_NOT_DEFINED
#define OS "unknown"
#endif
#define FooPOD_OS OS
  int i;
};
struct FooC
{
private:
  int i;
};
extern int a;
static int innerFunc();
int a = innerFunc();
int innerFunc() { return 5; }

void foo::Foo::innerFunc()
{
label1:
  int continuation = 0xCD + 0xFD + 0xBAADF00D + 0xDEADBEEF;
  auto la = [](int i1, int i2) -> bool mutable
  {
    label2:
      return i1 < i2;
  }(1, 2);
}

// Spaces

#include <stdio.h>
#define min(a, b) ((a) < (b) ? (a) : (b))

template <typename T, typename M>
inline T const &Min(T const &a, M const &b)
{
  return a < b ? a : b;
}

template <typename T>
class list
{
};
template <typename K, typename V = list<K>>
class hash
{
};
template <class T>
struct FooT
{
  char g();
  hash<int, list<char>> elems;
  template <int N>
  int foo() { return N; }
  template <>
  int foo<2>() { return Min<>(1, 5); }
  list<int> mem = {1, 2, 3};
  float vector[3] = {};
  FooT() : elems{{-1, {'c', 'p', 'p'}}, {1, {'j', 'b'}}}, vector{1f, 2f, 3f} {}
  FooT operator++(int) volatile { return *this; }
  auto f(T t) -> decltype(t + g()) { return t + g(); }
};

class Bar
{
};
struct FooBase
{
};
int doSomething(...) { return 1; }

struct Foo : private FooBase
{
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

namespace fooNS
{
  class FooClass : Foo, virtual FooBase
  {
    typedef int (FooClass::*ACTION)(int);

  public:
    FooClass() { act = &FooClass::nv_action; }
    virtual ~FooClass(){};
    int nv_action(int arg) { return arg; }
    virtual int action(int color, char alpha, float);
    virtual Foo *getSelf() { return Foo::getSelf(); }
    int method()
    {
      FooT<int> bar;
      int X[] = {1, 3, 5, 6, 7, 87, 1213, 2};
      int W[][3] = {{1, 3, 5}, {6, 7, 8}};
      int y = 0, x;
      auto la = [X, W](int i1, int i2) -> bool mutable
      { return i1 < i2; }(1, 2);
      auto laF = []() {};
      auto &[bb, cc, dd] = W[0];
      bool z = (bar.foo<2>() & 4) == 4;
      for (int z : {1, 2, 3})
      {
      }
      for (int i = 0; i < x; i++)
      {
        y += (y ^ 0x123) << 2;
      }
      do
      {
        try
        {
          if (0 < x && x < 10)
          {
            while (x != y)
            {
              x = min(x * 3, 5);
            }
            x = x >= 5 ?: 5;
          }
          else if (min(1, 5) == 1)
          {
            switch (this->action(0xFeeL, 0120, 0.01F))
            {
            default:
              break;
            case 1:
              continue;
            }
          }
        }
        catch (char *message)
        {
          const int *arr = X;
          x = ((y >= 0) ? arr[y] : -1);
        }
      } while (true);
    }
    ACTION act;

  private:
    int var;
  };
} // namespace fooNS

int fooNS::FooClass::action(int color, char alpha, float)
{
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
int &refTest(X &&x, int y, int b, void *(*)())
{
  int **&p = (int **&)x;
  int static &r = *&x;
  return r && (r & x) ? r : x;
}

struct fooS
{
  int i : 4;
  char j;
} foo_t;
enum fooE
{
  SUNDAY = 111,
  MONDAY = 222,
  TUESDAY = 333,
  WEDNESDAY = TUESDAY + 1
} foo_e;

// Wrapping and Braces

#include <stdio.h>
#define min(a, b) ((a) < (b) ? (a) : (b))

template <class T>
class list
{
};

class Bar
{
};
struct FooBase
{
};
int doSomething(...);
int doSomethingElse(...) { return 2; }

struct Foo : private FooBase
{
public:
  int i;
  virtual int action(int, char, float) = 0;
  virtual Foo *getSelf() { return this; }

private:
  static int method(){};
  list<Bar> bar;
};

namespace fooNS
{
  class FooClass : Foo, virtual FooBase
  {
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
} // namespace fooNS

int fooNS::FooClass::action(int color, char alpha, float)
{
  return doSomething(color);
}

namespace A
{
  namespace B
  {
    typedef void(fn)(int i, int j, int k);
    typedef void (*block)(int i, int j, int k);
  } // namespace B
} // namespace A
typedef int X;
int &refTest(X &&x)
{
  int **&p = (int **&)x;
  int static &r = *&x;
  return r && (r & x) ? r : x;
}

// todo something
void doSomething(int y, int b, void *(*)())
{
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
  if (1)
  {
    doSomething();
  }
  else if (2)
  {
    doSomething();
  }
  else
    doSomething();
  for (int i = 1, j = 2; i <= j; i++, j--)
    doSomethingElse();
  while (1)
    doSomethingElse();
  do
    doSomethingElse();
  while (1);
  switch (1)
  {
  case 0:
    return;
  case 1:
  {
    return;
  }
  }

  try
  {
    doSomethingElse();
  }
  catch (char *message)
  {
    return;
  }
}

struct fooS
{
  int i : 4;
  char j;
} foo_t;
enum fooE
{
  SUNDAY = 111,
  MONDAY = 222,
  TUESDAY = 333,
  WEDNESDAY = TUESDAY + 1
} foo_e;

template <typename T, typename M>
inline T const &Min(T const &a, M const &b)
{
  return a < b ? a : b;
}

template <typename K, typename V = list<K>>
class hash
{
};
template <class T>
struct FooT
{
  hash<int, list<char>> elems;
  template <int N>
  int foo() { return N; }
  template <>
  int foo<2>() { return Min<>(1, 5); }
  list<int> mem = {1, 2, 3};
  float vector[3];
  FooT() : elems{{-1, {'c', 'p', 'p'}}, {1, {'j', 'b'}}}, vector{1f, 2f, 3f}
  {
    auto la = [=, this, &mem, elems](int i1, int i2) -> bool mutable
    {
      return i1 < i2;
    }(1, 2);
    auto &[i, c, d] = vector;
  }
  auto f(T t) -> decltype(t + doSomething()) { return t + doSomething(); }
};

// Blank Lines

#include <vector>
class Foo
{
  friend class AnotherClass;

private:
  Foo *field1;

public:
  int field2;
  Foo() { field1 = new Foo(); }
  class InnerClass
  {
  public:
    static int x;
    static int y;
    int f();
    int g();
  };
};
int Foo::InnerClass::x = 25;
int Foo::InnerClass::f() { return 0; };
typedef Foo::InnerClass owner; // define a typedef
int owner::y = 11;             // use typedef with ::
int owner::g() { return 0; };
class AnotherClass
{
};
struct TestInterface
{
  static const int MAX = 42;
  static const int MIN = 0;
  virtual void method1() = 0;
  virtual void method2() = 0;
};

#include <Core/Math/Math.hpp>
#include <cstring>
namespace Ra
{
    namespace Core
    {
        namespace Math
        {
            inline constexpr Scalar toRadians( Scalar a )
            {
                return toRad * a;
            }

            inline constexpr Scalar toDegrees( Scalar a )
            {
                return toDeg * a ;
            }

            template<typename T> inline T ipow( const T& x, uint exp )
            {
                if ( exp == 0 )
                {
                    return T( 1 );
                }
                if ( exp == 1 )
                {
                    return x;
                }
                T p = ipow( x, exp / 2 );
                if ( ( exp  % 2 ) == 0 )
                {
                    return p * p;
                }
                else
                {
                    return p * p * x;
                }
            }

            /// This helper class is needed because C++ doesn't support function template
            /// partial specialization.
            namespace
            {
                template<typename T, uint N>
                struct IpowHelper
                {
                    static inline constexpr T pow( const T& x )
                    {
                        return ( N % 2 == 0 ) ? IpowHelper < T, N / 2 >::pow( x ) * IpowHelper < T, N / 2 >::pow( x )
                                              : IpowHelper < T, N / 2 >::pow( x ) * IpowHelper < T, N / 2 >::pow( x ) * x;
                    }
                };

                template<typename T>
                struct IpowHelper<T, 1>
                {
                    static inline constexpr T pow( const T& x )
                    {
                        return x;
                    }
                };

                template<typename T>
                struct IpowHelper<T, 0>
                {
                    static inline constexpr T pow( const T& x )
                    {
                        return T( 1 );
                    }
                };

            }

            // Nb : T is last for automatic template argument deduction.
            template <uint N, typename T>
            inline constexpr T ipow( const T& x )
            {
                return IpowHelper<T, N>::pow( x );
            }

            // Signum implementation that works for unsigned types.
            template <typename T> inline constexpr
            int signum(T x, std::false_type is_signed)
            {
                return T(0) < x;
            }

            template <typename T> inline constexpr
            int signum(T x, std::true_type is_signed)
            {
                return (T(0) < x) - (x < T(0));
            }

            template <typename T>
            inline constexpr int sign( const T& val )
            {
                return signum( val, std::is_signed<T>() );
            }

            template <typename T>
            inline constexpr T signNZ( const T& val )
            {
                return T(std::copysign( T(1), val) );
            }

            template <typename T>
            inline constexpr T clamp( T v, T min, T max )
            {
                return std::max( min, std::min( v, max ) );
            }

            template <typename T>
            inline constexpr T saturate( T v )
            {
                return clamp( v, static_cast<T>(0), static_cast<T>(1) );
            }

            inline bool areApproxEqual(Scalar a, Scalar b, Scalar eps)
            {
               return std::abs(b-a) < eps;
            }

            template <typename T>
            constexpr T lerp(const T& a, const T& b, Scalar t)
            {
                return (1-t) * a + t * b;
            }

            // Magic constants for fast inverse square root.
            // Reference  : Fast Inverse Square Root, C. Lomont, 2003 Tech.Rep (Purdue)

            template< typename T> struct FastInvSqrt {};
            // Quake 3's constant for float. Lomont also recommends 0x5f375a86
            template<> struct FastInvSqrt<float>
            {
                static constexpr std::uint32_t magic{ 0x5f3759df };
                inline static void doMagic( float* RESTRICT x )
                {
                    static_assert(sizeof(float) == sizeof(std::uint32_t));
                    std::uint32_t i;
                    // Use memcpy instead of a cast (it's the C++ way !)
                    std::memcpy(&i, x, sizeof(float));
                    i = magic - (i >>1);
                    std::memcpy(x, &i, sizeof(float));
                }
            };
            template<> struct FastInvSqrt<double>
            {
                // Lomont's constant for double
                static constexpr std::uint64_t magic{ 0x5fe6ec85e7de30da };
                inline static void doMagic( double* RESTRICT x )
                {
                    static_assert(sizeof(double) == sizeof(std::uint64_t));
                    std::uint64_t i;
                    std::memcpy(&i, x, sizeof(double));
                    i = magic - (i >>1);
                    std::memcpy(x, &i, sizeof(double));
                }
            };

            inline Scalar fastInvSqrt(Scalar x)
            {
                const Scalar x2 = x * Scalar(0.5);
                FastInvSqrt<Scalar>::doMagic(&x); // Carmack's Magic Trick !
                return x* (Scalar(1.5) - x2*x*x); // Newton iteration
            }

        }
    }
}

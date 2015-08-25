#ifndef RADIUMENGINE_DEBUG_DISPLAY_HPP_
#define RADIUMENGINE_DEBUG_DISPLAY_HPP_

/// This file's main purpose is to declare a set of macros to allow drawing
/// primitives from everywhere in the code, for debug purposes.
/// These macros can be completely disabled by #defining the following constant :

// define RA_DISABLE_DEBUG_DISPLAY
#ifndef RA_DISABLE_DEBUG_DISPLAY

#include <Core/Utils/Singleton.hpp>
#include <Engine/Entity/Entity.hpp>
#include <Engine/Entity/Component.hpp>
#include <Engine/Renderer/RenderObject/Primitives/DrawPrimitives.hpp>

namespace Ra
{
    namespace Engine
    {
        class RenderObject;

        /// Component for debug drawing. @see DebugEntity.
        class RA_API DebugComponent : public Component
        {
        public:
            DebugComponent();
            void initialize() override {}
            void addDebugDrawable( RenderObject* ro );
        private:
            std::vector<Core::Index> m_debugDrawableIndices;
        };

        /// This entity allows to add debug drawables from everywhere in the code.
        /// It should have only one component and its transform should not change.
        class RA_API DebugEntity: public Entity, public Core::Singleton<DebugEntity>
        {
        public:
            DebugEntity();

            /// Access to its one component.
            DebugComponent* getDebugComponent()  const;

            /// Shortcut to getInstance->getDebugComponent();
            static inline DebugComponent* dbgCmp ()
            {
                return getInstanceRef().getDebugComponent();
            }

            // Override Editable interface to disable property editing
            virtual void getProperties( Core::AlignedStdVector<EditableProperty>& entityPropsOut ) const override {}
            virtual void setProperty( const EditableProperty& prop ) override {}

        };

    }
}
/// Macros for debug drawing. All coordinates are in world space.
#define RA_DISPLAY_POINT( p, color, scale )     Ra::Engine::DebugEntity::dbgCmp()->addDebugDrawable( Ra::Engine::DrawPrimitives::Point(Ra::Engine::DebugEntity::dbgCmp(), p, color, scale ))
#define RA_DISPLAY_VECTOR( p, v, color )        Ra::Engine::DebugEntity::dbgCmp()->addDebugDrawable( Ra::Engine::DrawPrimitives::Vector(Ra::Engine::DebugEntity::dbgCmp(), p, v, color ))
#define RA_DISPLAY_RAY( r, color )              Ra::Engine::DebugEntity::dbgCmp()->addDebugDrawable( Ra::Engine::DrawPrimitives::Ray(Ra::Engine::DebugEntity::dbgCmp(), r, color ))
#define RA_DISPLAY_TRIANGLE( a, b, c, color )   Ra::Engine::DebugEntity::dbgCmp()->addDebugDrawable( Ra::Engine::DrawPrimitives::Triangle(Ra::Engine::DebugEntity::dbgCmp(), a, b, c, color ))
#define RA_DISPLAY_NORMAL( p, n, color, scale ) Ra::Engine::DebugEntity::dbgCmp()->addDebugDrawable( Ra::Engine::DrawPrimitives::Normal(Ra::Engine::DebugEntity::dbgCmp(), p, n, color, scale ))
#define RA_DISPLAY_FRAME( t, scale )            Ra::Engine::DebugEntity::dbgCmp()->addDebugDrawable( Ra::Engine::DrawPrimitives::Frame(Ra::Engine::DebugEntity::dbgCmp(), t, scale ))

#else // if debug display is disabled

#define RA_DISPLAY_POINT( p, color, scale )     // Nothing
#define RA_DISPLAY_VECTOR( p, v, color )        // ...
#define RA_DISPLAY_RAY( r, color )              // ...
#define RA_DISPLAY_TRIANGLE( a, b, c, color )   // ...
#define RA_DISPLAY_NORMAL( p, n, color, scale ) // ...
#define RA_DISPLAY_FRAME( t, scale )            // ...

#endif //!defined DISABLED_DEBUG_DISPLAY

#endif //RADIUMENGINE_DEBUG_DISPLAY_HPP_
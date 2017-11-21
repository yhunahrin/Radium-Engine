#include <FancyMeshPlugin.hpp>

#include <Engine/RadiumEngine.hpp>

#include <FancyMeshSystem.hpp>

#include <QApplication>

namespace FancyMeshPlugin
{

    FancyMeshPluginC::~FancyMeshPluginC()
    {
    }

    void FancyMeshPluginC::registerPlugin( const Ra::PluginContext& context )
    {
        m_system = new FancyMeshSystem;
        context.m_engine->registerSystem( "FancyMeshSystem", m_system );

        m_textureAction = new QAction();
        m_textureAction->setCheckable(true);
        m_textureAction->setChecked(true);
        m_textureAction->setShortcut( QApplication::translate("MainWindow", "T", Q_NULLPTR) );
        connect( m_textureAction, &QAction::toggled, this, &FancyMeshPluginC::triggerTexture );

        m_subdivWireframeAction = new QAction();
        m_subdivWireframeAction->setCheckable(true);
        m_subdivWireframeAction->setChecked(true);
        m_subdivWireframeAction->setShortcut( QApplication::translate("MainWindow", "W", Q_NULLPTR) );
        connect( m_subdivWireframeAction, &QAction::toggled, this, &FancyMeshPluginC::triggerSubdivWireframe );
    }

    bool FancyMeshPluginC::doAddWidget( QString &name )
    {
        return false;
    }

    QWidget* FancyMeshPluginC::getWidget()
    {
        return nullptr;
    }

    bool FancyMeshPluginC::doAddMenu()
    {
        return false;
    }

    QMenu* FancyMeshPluginC::getMenu()
    {
        return nullptr;
    }

    bool FancyMeshPluginC::doAddAction( int& nb )
    {
        nb = 2;
        return true;
    }

    QAction* FancyMeshPluginC::getAction( int id )
    {
        if (id == 0)
        {
            return m_textureAction;
        }
        if (id == 1)
        {
            return m_subdivWireframeAction;
        }
        return nullptr;
    }

    void FancyMeshPluginC::triggerTexture( bool trigger )
    {
        m_system->triggerTexture( trigger );
    }

    void FancyMeshPluginC::triggerSubdivWireframe( bool trigger )
    {
        m_system->triggerSubdivWireframe( trigger );
    }
}

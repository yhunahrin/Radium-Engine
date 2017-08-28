#include <chrono>
#include <memory>
#include <vector>

#include <QApplication>

#include <Core/Time/Timer.hpp>
#include <GuiBase/TimerData/FrameTimerData.hpp>
#include <GuiBase/Viewer/Viewer.hpp>

class QTimer;
namespace Ra
{
    namespace Core
    {
        class TaskQueue;
    }
}

namespace Ra
{
    namespace Engine
    {
        class RadiumEngine;
    }
}

namespace Ra
{
    namespace Gui
    {
        class Viewer;
        class MainWindow;
    }
}

/// Allow singleton-like access to the main app à la qApp.
#if defined(mainApp)
#undef mainApp
#endif
#define mainApp (static_cast<Ra::BaseApplication*>(qApp))

namespace Ra
{
    /// This class contains the main application logic. It owns the engine and the GUI.
    class BaseApplication : public QApplication
    {
        Q_OBJECT

    public:
        BaseApplication( int argc, char** argv, QString applicationName = "RadiumEngine", QString organizationName = "STORM-IRIT" );
        ~BaseApplication();

        /// Advance the engine for one frame.
        void radiumFrame();

        bool isRunning() const { return !m_isAboutToQuit; }

        const Engine::RadiumEngine* getEngine () const { return m_engine.get();}

        uint getFrameCount() const { return m_frameCounter; }

        const std::string& getExportFolder() const {return m_exportFolderName;}

    signals:
        /// Fired when the engine has just started, before the frame timer is set.
        void starting();

        /// Fired when the engine is about to stop.
        void stopping();

        /// Fired when the scene has changed.
        void sceneChanged( const Core::Aabb& );

        void updateFrameStats( const std::vector<FrameTimerData>& );

        void loadComplete();

        void selectedItem(const Ra::Engine::ItemEntry& entry);

    public slots:

        void loadFile( QString path, bool fitCam = true );
        void framesCountForStatsChanged( uint count );
        void appNeedsToQuit();
        void setRealFrameRate( bool on );
        void setRecordFrames( bool on );
        void setRecordTimings( bool on );
        void setRecordGraph( bool on );
        void setRecordMeshes( bool on );

        void recordFrame();
        void recordMeshes();

        void onSelectedItem(const Ra::Engine::ItemEntry& entry) { emit selectedItem(entry); }

    protected:
        /// Create signal / slots connections
        void createConnections();

        /// Load plugins from the specified folder.
        /// If loadList is empty, attempts to load all DLLs in the folder (except those on the ignore list)
        /// If loadList contains names it will only look for DLLs in that folder with the given name.
        bool loadPlugins( const std::string& pluginsPath, const QStringList& loadList, const QStringList& ignoreList );

        void setupScene();
        void addBasicShaders();


        // Public variables, accessible through the mainApp singleton.
    public:
        /// Application main window and GUI root class.
        std::unique_ptr<Gui::MainWindow> m_mainWindow;

        /// Instance of the radium engine.
        std::unique_ptr<Engine::RadiumEngine> m_engine;

        /// Task queue for processing tasks.
        std::unique_ptr<Core::TaskQueue> m_taskQueue;

        /// Number of frames per second to generate.
        uint m_targetFPS;

    private:
        /// Pointer to OpenGL Viewer for render call (belongs to MainWindow).
        Gui::Viewer* m_viewer;

        /// Timer to wake us up at every frame start.
        QTimer* m_frameTimer;

        /// Time since the last frame start.
        Core::Timer::TimePoint m_lastFrameStart;

        // Frame count
        /// How many frames have been run
        uint m_frameCounter;
        /// Frequency for update of frame timings
        uint m_frameCountBeforeUpdate;
        /// Number of frames to run before exiting. If 0, run forever.
        uint m_numFrames;

        /// Maximum number of threads on which the app should run
        uint m_maxThreads;
        /// Storage for the frame timings
        std::vector<FrameTimerData> m_timerData;

        /// If true, use the wall clock to advance the engine. If false, use a fixed time step.
        bool m_realFrameRate;

        // Options to control monitoring and outputs
        /// Name of the folder where exported data goes
        std::string m_exportFolderName;

        /// If true, dump each frame to a PNG file.
        bool m_recordFrames;
        /// If true, print the detailed timings of each frame
        bool m_recordTimings;
        /// If true, print the task graph;
        bool m_recordGraph;
        /// If true, export visible meshes at every frames
        bool m_recordMeshes;

        bool m_isAboutToQuit;
    };
}

namespace Ra
{
    class MainApplication : public BaseApplication
    {
    public:
        using BaseApplication::BaseApplication;

    };

}

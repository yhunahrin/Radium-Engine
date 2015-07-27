#include <Engine/Renderer/FancyMeshPlugin/FancyMeshComponent.hpp>

#include <Core/String/StringUtils.hpp>
#include <Engine/Renderer/Drawable/DrawableManager.hpp>
#include <Engine/Renderer/FancyMeshPlugin/FancyMeshDrawable.hpp>
#include <Engine/Renderer/Mesh/Mesh.hpp>
#include <Engine/Renderer/Material/Material.hpp>

namespace Ra
{

Engine::FancyMeshComponent::FancyMeshComponent(const std::string& name)
    : Component(name)
{
}

Engine::FancyMeshComponent::~FancyMeshComponent()
{
	// TODO(Charly): Should we ask the drawable manager to delete our drawable ?
	m_drawableManager->removeDrawable(m_drawable);
}

void Engine::FancyMeshComponent::initialize()
{
}

void Engine::FancyMeshComponent::addMeshDrawable( const Core::TriangleMesh& mesh, const std::string& name )
{
	FancyMeshDrawable* drawable = new FancyMeshDrawable(name);
	drawable->setComponent(this);
	drawable->setVisible(true);

    Mesh* displayMesh = new Mesh(name);
    displayMesh->loadGeometry(mesh);
    drawable->addMesh(displayMesh);
    drawable->setMaterial(new Material("Default"));
    m_drawable = m_drawableManager->addDrawable(drawable);
}

void Engine::FancyMeshComponent::handleMeshLoading(const FancyComponentData& data)
{
	FancyMeshDrawable* drawable = new FancyMeshDrawable(data.name);
	drawable->setComponent(this);
	drawable->setVisible(true);

	for (uint i = 0; i < data.meshes.size(); ++i)
	{
		FancyMeshData meshData = data.meshes[i];

		std::stringstream ss;
		ss << data.name << "_mesh_" << i;
		std::string meshName = ss.str();

		Mesh* mesh = new Mesh(meshName);
		mesh->loadGeometry(meshData.mesh, meshData.tangents, 
						   meshData.bitangents, meshData.texcoords);
		drawable->addMesh(mesh);
	}

	drawable->setMaterial(data.material);

	m_drawable = m_drawableManager->addDrawable(drawable);
}

}
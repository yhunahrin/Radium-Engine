#include <Engine/Renderer/Texture/TextureManager.hpp>

#include <cstdio>

#include <FreeImage.h>

#include <Engine/Renderer/Texture/Texture.hpp>

namespace Ra
{

Engine::TextureManager::TextureManager()
{
}

Engine::TextureManager::~TextureManager()
{
    for (auto& tex : m_textures)
    {
        delete tex.second;
    }
    m_textures.clear();
}

Engine::Texture* Engine::TextureManager::addTexture(const std::string& filename)
{
    Texture* ret = nullptr;

    FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
    FIBITMAP* dib = nullptr; // TODO This name is nonsense, change it

    // Find format from file signature
    fif = FreeImage_GetFileType(filename.c_str(), 0);

    if (FIF_UNKNOWN == fif)
    {
        // Find format from file extension
        fif = FreeImage_GetFIFFromFilename(filename.c_str());
    }

    if (FIF_UNKNOWN == fif)
    {
        // Still unknown
        std::string error = "Cannot determine " + filename + " image format.";
        fprintf(stderr, "%s\n", error.c_str());
        CORE_ASSERT(0, error.c_str());

        return nullptr;
    }

    if (FreeImage_FIFSupportsReading(fif))
    {
        dib = FreeImage_Load(fif, filename.c_str());
    }

    std::string error = "Something went wrong while trying to load " + filename + ".";
    //CORE_ASSERT(dib, error.c_str());

    if (nullptr == dib)
    {
        fprintf(stderr, "%s\n", error.c_str());
        return nullptr;
    }

    ret = new Texture(filename, GL_TEXTURE_2D);
    unsigned char* data = FreeImage_GetBits(dib);

    int bpp = FreeImage_GetBPP(dib);
    int format = (bpp == 24 ? GL_BGR : 0);   // TODO Handle other formats
    int internal = (bpp == 24 ? GL_RGB : 0); // TODO Handle other formats
    int w = FreeImage_GetWidth(dib);
    int h = FreeImage_GetHeight(dib);

    fprintf(stderr,
            "Image stats (%s) :\n\tBPP    : %08x\n"
            "\tFormat : %08x\n\tSize   : %ux%u\n",
            filename.c_str(), bpp, format, w, h);

    CORE_ASSERT(data, "Data is null");

    ret->initGL(internal, w, h, format, GL_UNSIGNED_BYTE, data);
    ret->genMipmap(GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR);

    m_textures.insert(TexturePair(filename, ret));

    FreeImage_Unload(dib);

    return ret;
}

Engine::Texture* Engine::TextureManager::getOrLoadTexture(const std::string& filename)
{
    Texture* ret = nullptr;
    auto it = m_textures.find(filename);

    if (it != m_textures.end())
    {
        ret = it->second;
    }
    else
    {
        ret = addTexture(filename);
    }

    return ret;
}

void Engine::TextureManager::deleteTexture(const std::string& filename)
{
    auto it = m_textures.find(filename);

    if (it != m_textures.end())
    {
        delete it->second;
        m_textures.erase(it);
    }
}

void Engine::TextureManager::deleteTexture(Texture* texture)
{
    deleteTexture(texture->getName());
}


} // namespace Ra
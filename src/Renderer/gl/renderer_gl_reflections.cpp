#include "renderer_gl.h"




MOSS_API Moss_ShadowMap* Moss_RendererCreateShadowMap(Moss_Renderer* renderer, const Moss_ShadowMapDesc* desc);
MOSS_API void Moss_RendererDestroyShadowMap(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_RendererBeginShadowPass(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_RendererEndShadowPass(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
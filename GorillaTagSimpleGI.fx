// ReShade effect for PC rendering. Requires the standard ReShade.fxh header.
// Screen-space GI uses the visible frame and the depth buffer ReShade provides.
#include "ReShade.fxh"

uniform bool HasSceneDepth < source = "bufready_depth"; >;

uniform float BounceStrength <
    ui_type = "slider"; ui_min = 0.0; ui_max = 1.0; ui_step = 0.01;
    ui_label = "Indirect lighting";
> = 0.36;

uniform float RayRadius <
    ui_type = "slider"; ui_min = 0.08; ui_max = 0.60; ui_step = 0.01;
    ui_label = "Ray radius";
    ui_tooltip = "Maximum ray length as a fraction of the surface distance.";
> = 0.32;

uniform float VerticalFOV <
    ui_type = "slider"; ui_min = 45.0; ui_max = 135.0; ui_step = 1.0;
    ui_label = "Approximate vertical FOV";
    ui_tooltip = "Match the vertical field of view of the image ReShade is processing.";
> = 90.0;

uniform float Contrast <
    ui_type = "slider"; ui_min = 1.0; ui_max = 1.25; ui_step = 0.005;
    ui_label = "Contrast";
> = 1.045;

#define GI_WIDTH ((BUFFER_WIDTH + 1) / 2)
#define GI_HEIGHT ((BUFFER_HEIGHT + 1) / 2)

texture2D IndirectTex
{
    Width = GI_WIDTH;
    Height = GI_HEIGHT;
    Format = RGBA16F;
};

sampler2D IndirectSampler
{
    Texture = IndirectTex;
    MinFilter = POINT;
    MagFilter = POINT;
    MipFilter = POINT;
};

float SceneDepth(float2 uv)
{
    float farPlane = RESHADE_DEPTH_LINEARIZATION_FAR_PLANE;
    return lerp(rcp(farPlane), 1.0, saturate(ReShade::GetLinearizedDepth(uv)));
}

float3 ViewPosition(float2 uv, float depth, float2 projectionScale)
{
    return float3((uv * 2.0 - 1.0) * projectionScale, 1.0) * depth;
}

float3 SurfaceNormal(float2 uv, float3 center, float2 projectionScale)
{
    float2 dx = float2(BUFFER_RCP_WIDTH, 0.0);
    float2 dy = float2(0.0, BUFFER_RCP_HEIGHT);
    float3 left = center - ViewPosition(uv - dx, SceneDepth(uv - dx), projectionScale);
    float3 right = ViewPosition(uv + dx, SceneDepth(uv + dx), projectionScale) - center;
    float3 up = center - ViewPosition(uv - dy, SceneDepth(uv - dy), projectionScale);
    float3 down = ViewPosition(uv + dy, SceneDepth(uv + dy), projectionScale) - center;

    // Favor the neighbor on the same surface at a depth discontinuity.
    float3 normal = normalize(cross(abs(up.z) < abs(down.z) ? up : down,
                                    abs(left.z) < abs(right.z) ? left : right));
    return dot(normal, center) > 0.0 ? -normal : normal;
}

float4 TraceIndirect(float4 position : SV_Position, float2 uv : TEXCOORD) : SV_Target
{
    if (!HasSceneDepth || BounceStrength <= 0.0)
        return 0.0;

    float depth = SceneDepth(uv);
    if (depth >= 0.995)
        return 0.0;

    float2 projectionScale = tan(radians(VerticalFOV) * 0.5)
                           * float2(BUFFER_ASPECT_RATIO, 1.0);
    float3 center = ViewPosition(uv, depth, projectionScale);
    float3 normal = SurfaceNormal(uv, center, projectionScale);
    float3 axis = abs(normal.z) < 0.9 ? float3(0.0, 0.0, 1.0) : float3(0.0, 1.0, 0.0);
    float3 tangent = normalize(cross(axis, normal));
    float3 bitangent = cross(normal, tangent);
    float range = min(depth * RayRadius, 0.05);
    float3 origin = center + normal * (range * 0.035);
    float4 indirect = 0.0;

    // Four cosine-weighted hemisphere directions with a stable per-pixel rotation.
    float rotation = 3.1415927 * dot(frac(floor(position.xy) * 0.5), float2(0.25, 0.5));
    [loop]
    for (int ray = 0; ray < 4; ++ray)
    {
        float angle = rotation + float(ray) * 2.3999632;
        float radial = (float(ray) + 0.5) * 0.25;
        float3 direction = sqrt(radial) * (cos(angle) * tangent + sin(angle) * bitangent)
                         + sqrt(1.0 - radial) * normal;
        float previousGap = -1.0;

        [loop]
        for (int step = 1; step <= 6; ++step)
        {
            float fraction = float(step) / 6.0;
            float3 samplePosition = origin + direction * (range * fraction);
            if (samplePosition.z <= 0.0)
                break;

            float2 hitUV = 0.5 + 0.5 * (samplePosition.xy / (samplePosition.z * projectionScale));
            if (any(hitUV <= 0.0) || any(hitUV >= 1.0))
                break;

            float hitDepth = SceneDepth(hitUV);
            float gap = samplePosition.z - hitDepth;
            if (gap >= 0.0 && previousGap < 0.0 && hitDepth < 0.995)
            {
                float thickness = range * (0.055 + abs(direction.z) / 6.0);
                if (gap < thickness)
                {
                    float weight = (1.0 - fraction * 0.75) * (1.0 - gap / thickness);
                    float3 incoming = saturate(tex2Dlod(ReShade::BackBuffer, float4(hitUV, 0, 0)).rgb);
                    indirect += float4(pow(incoming, 2.2), 1.0) * weight;
                }
                break;
            }
            previousGap = gap;
        }
    }

    return indirect * (0.25 * (1.0 - smoothstep(0.35, 0.85, depth)));
}

float4 CompositeGI(float4 position : SV_Position, float2 uv : TEXCOORD) : SV_Target
{
    float3 color = tex2D(ReShade::BackBuffer, uv).rgb;
    if (HasSceneDepth && BounceStrength > 0.0)
    {
        float depth = SceneDepth(uv);
        if (depth < 0.995)
        {
            const float2 size = float2(GI_WIDTH, GI_HEIGHT);
            float2 grid = uv * size - 0.5;
            float2 base = floor(grid);
            float2 blend = frac(grid);
            float4 indirect = 0.0;
            float weightSum = 0.0;

            // Bilinear upsampling with depth weights to keep bounce off foreground edges.
            [unroll]
            for (int tap = 0; tap < 4; ++tap)
            {
                float2 offset = float2(tap % 2, tap / 2);
                float2 sampleUV = (clamp(base + offset, 0.0, size - 1.0) + 0.5) / size;
                float2 spatial = lerp(1.0 - blend, blend, offset);
                float depthWeight = saturate(1.0 - abs(SceneDepth(sampleUV) - depth) / (depth * 0.04));
                float weight = spatial.x * spatial.y * depthWeight;
                indirect += tex2Dlod(IndirectSampler, float4(sampleUV, 0, 0)) * weight;
                weightSum += weight;
            }

            indirect /= max(weightSum, 0.05);
            float3 linearColor = pow(saturate(color), 2.2);
            // Modulate the bounced light by the receiver color and softly darken contacts.
            linearColor *= 1.0 - BounceStrength * 0.16 * indirect.a;
            linearColor += BounceStrength * indirect.rgb * lerp(0.18, 0.75, sqrt(linearColor));
            color = pow(saturate(linearColor), 1.0 / 2.2);
        }
    }

    return float4(saturate((color - 0.5) * Contrast + 0.5), 1.0);
}

technique GorillaTagSimpleGI <
    ui_label = "Gorilla Tag Simple GI";
    ui_tooltip = "Screen-space bounce lighting with a subtle contrast boost. Contrast also works without depth.";
>
{
    pass Trace
    {
        VertexShader = PostProcessVS;
        PixelShader = TraceIndirect;
        RenderTarget = IndirectTex;
    }
    pass Composite
    {
        VertexShader = PostProcessVS;
        PixelShader = CompositeGI;
    }
}

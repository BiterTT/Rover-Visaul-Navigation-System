Shader "Custom/DepthToFloat"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            ZWrite Off
            ZTest Always
            Cull Off
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _CameraDepthTexture;

            struct v2f {
                float4 pos : SV_POSITION;
                float2 uv : TEXCOORD0;
            };

            v2f vert(appdata_base v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = v.texcoord;
                return o;
            }

            float4 frag(v2f i) : SV_Target
            {
                float rawZ = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);
                float linearDepth = LinearEyeDepth(rawZ); // 使用 Unity 自带函数
                return float4(linearDepth, 0, 0, 1); // 输出 float32 深度到 R 通道
            }
            ENDCG
        }
    }
    FallBack Off
}




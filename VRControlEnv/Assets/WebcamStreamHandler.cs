using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
//using RosImage = RosMessageTypes.ROSTCPEndpoint.ImageMsg;
using RosCompressedImage = RosMessageTypes.ROSTCPEndpoint.CompressedImageMsg;

public class WebcamStreamHandler : MonoBehaviour
{
    public int inputWidth = 4096;
    public int inputHeight = 2048;

    public bool needsScaling = false;
    public int scaleWidth = 4096;
    public int scaleHeight = 2048;
    public RenderTexture outputTexture;
    private Texture2D bufferTexture;
    private int xOffset;
    private int yOffset;
    private bool needsBuffer;

    Texture2D tex;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<RosCompressedImage>("video_frames", FrameRecived);
        tex = new Texture2D(inputWidth, inputHeight, TextureFormat.RGB24, false);
        bufferTexture = new Texture2D(4096, 2048, TextureFormat.RGB24, false);
        //Only use buffer texture if resolutions dont match
        needsBuffer = !(inputWidth == 4096 && inputHeight == 2048);

        //Place image in the center of the output texture
        if (needsBuffer)
        {
            xOffset = (4096 - inputWidth) / 2;
            yOffset = (2048 - inputHeight) / 2;
        }
        Debug.Log("init");
    }

    void FrameRecived(RosCompressedImage frame)
    {
        tex.LoadImage(frame.data);
        tex.Apply();
        if (needsBuffer || needsScaling)
        {
            if (tex.width > 4096)
            {
                xOffset = (tex.width-4096) / 2;
                yOffset = (2048 - tex.height) / 2;
                Graphics.CopyTexture(tex, 0, 0, xOffset, 0, tex.width-(xOffset*2), tex.height, bufferTexture, 0, 0, 0, yOffset);
                Graphics.Blit(bufferTexture, outputTexture);
            }
            else
            {
                if (needsScaling) scale(tex, scaleWidth, scaleHeight);
                xOffset = (4096 - tex.width) / 2;
                yOffset = (2048 - tex.height) / 2;
                Graphics.CopyTexture(tex, 0, 0, 0, 0, tex.width, tex.height, bufferTexture, 0, 0, xOffset, yOffset);
                Graphics.Blit(bufferTexture, outputTexture);
                //Graphics.CopyTexture(tex, 0, 0, 0, 0, tex.width, tex.height, bufferTexture, 0, 0, xOffset, yOffset);
            }
            
            
        }
        else
        {
            if (needsScaling) scale(tex, scaleWidth, scaleHeight);
            Graphics.Blit(tex, outputTexture);
        }
        
        //Graphics.Blit(tex, rt);
    }


///Origional Author: https://pastebin.com/qkkhWs2J
/// A unility class with functions to scale Texture2D Data.
///
/// Scale is performed on the GPU using RTT, so it's blazing fast.
/// Setting up and Getting back the texture data is the bottleneck. 
/// But Scaling itself costs only 1 draw call and 1 RTT State setup!
/// WARNING: This script override the RTT Setup! (It sets a RTT!)    
///
/// Note: This scaler does NOT support aspect ratio based scaling. You will have to do it yourself!
/// It supports Alpha, but you will have to divide by alpha in your shaders, 
/// because of premultiplied alpha effect. Or you should use blend modes.



    /// <summary>
    /// Returns a scaled copy of given texture. 
    /// </summary>
    /// <param name="tex">Source texure to scale</param>
    /// <param name="width">Destination texture width</param>
    /// <param name="height">Destination texture height</param>
    /// <param name="mode">Filtering mode</param>
    public static Texture2D scaled(Texture2D src, int width, int height, FilterMode mode = FilterMode.Trilinear)
    {
        Rect texR = new Rect(0, 0, width, height);
        RenderTexture rtt = new RenderTexture(width, height, 32);
        _gpu_scale(src, rtt, width, height, mode);

        //Get rendered data back to a new texture
        Texture2D result = new Texture2D(width, height, TextureFormat.ARGB32, true);
        result.Reinitialize(width, height);
        result.ReadPixels(texR, 0, 0, true);

        return result;
    }

    static RenderTexture rtt;

    /// <summary>
    /// Scales the texture data of the given texture.
    /// </summary>
    /// <param name="tex">Texure to scale</param>
    /// <param name="width">New width</param>
    /// <param name="height">New height</param>
    /// <param name="mode">Filtering mode</param>
    public static void scale(Texture2D tex, int width, int height, FilterMode mode = FilterMode.Trilinear)
    {
        if (rtt is null)
        {
            rtt = new RenderTexture(width, height, 32);
        } else
        {
            if (rtt.width == width && rtt.height == height)
            {

            }else
            {
                rtt.Release();
                rtt = new RenderTexture(width, height, 32);
            }
        }
        Rect texR = new Rect(0, 0, width, height);
        
        _gpu_scale(tex, rtt, width, height, mode);

        // Update new texture
        tex.Reinitialize(width, height);
        tex.ReadPixels(texR, 0, 0, true);
        tex.Apply(true);    //Remove this if you hate us applying textures for you :)
  
    }

    // Internal unility that renders the source texture into the RTT - the scaling method itself.
    static void _gpu_scale(Texture2D src, RenderTexture rtt, int width, int height, FilterMode fmode)
    {
        //We need the source texture in VRAM because we render with it
        src.filterMode = fmode;
        src.Apply(true);

        //Using RTT for best quality and performance. Thanks, Unity 5
        //RenderTexture rtt = new RenderTexture(width, height, 32);

        //Set the RTT in order to render to it
        Graphics.SetRenderTarget(rtt);

        //Setup 2D matrix in range 0..1, so nobody needs to care about sized
        GL.LoadPixelMatrix(0, 1, 1, 0);

        //Then clear & draw the texture to fill the entire RTT.
        GL.Clear(true, true, new Color(0, 0, 0, 0));
        Graphics.DrawTexture(new Rect(0, 0, 1, 1), src);
    }

}
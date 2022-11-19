using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
//using RosImage = RosMessageTypes.ROSTCPEndpoint.ImageMsg;
using RosCompressedImage = RosMessageTypes.ROSTCPEndpoint.CompressedImageMsg;

public class WebcamStreamHandler : MonoBehaviour
{
    public int inputWidth = 4096;
    public int inputHeight = 2048;

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
        if (needsBuffer)
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
                xOffset = (4096 - tex.width) / 2;
                yOffset = (2048 - tex.height) / 2;
                Graphics.CopyTexture(tex, 0, 0, 0, 0, tex.width, tex.height, bufferTexture, 0, 0, xOffset, yOffset);
                Graphics.Blit(bufferTexture, outputTexture);
            }
            
            
        }
        else
        {
            
            Graphics.Blit(tex, outputTexture);
        }
        
        //Graphics.Blit(tex, rt);
    }
}
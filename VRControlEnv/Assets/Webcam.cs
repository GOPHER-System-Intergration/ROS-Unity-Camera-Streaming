using UnityEngine;

public class Webcam : MonoBehaviour {
    //Numeric ID for USB  webcam device
    public int deviceId = 0;
    //Input webcam dimensions
    public int inputWidth = 4096;
    public int inputHeight = 2048;
    //Render texture to write webcam image to
    public RenderTexture outputTexture;
    private WebCamTexture webcamTexture;
    //Buffer texture to be used if the input image is not a 360 resolution
    private Texture2D bufferTexture;
    private int xOffset;
    private int yOffset;
    private bool needsBuffer;
	void Start () {
        WebCamDevice[] devices = WebCamTexture.devices;
        webcamTexture = new WebCamTexture(3840,1080);
        bufferTexture = new Texture2D(4096, 2048);

        //Only use buffer texture if resolutions dont match
        needsBuffer = !(inputWidth==4096&&inputHeight==2048);

        //Place image in the center of the output texture
        if (needsBuffer) {
            xOffset = (4096-inputWidth)/2;
            yOffset = (2048-inputHeight)/2;
        }
        
        webcamTexture.deviceName = devices[deviceId].name;
        
        webcamTexture.Play();
        
    }

    void Update() {
        //Copy webcam texture to render texture output
        if (needsBuffer) {
            Graphics.CopyTexture(webcamTexture, 0, 0, 0, 0, inputWidth, inputHeight, bufferTexture, 0, 0, xOffset, yOffset);
            Graphics.Blit(bufferTexture, outputTexture);
        } else {
            Graphics.Blit(webcamTexture, outputTexture);
        }
        
    }
}

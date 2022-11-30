using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using System;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR;
//using RosImage = RosMessageTypes.ROSTCPEndpoint.ImageMsg;
using RosConsoleMessage = RosMessageTypes.ROSTCPEndpoint.ConsoleInfoMsg;

public class ConsoleHandler : MonoBehaviour
{
    public Canvas canvas;
    public Text firstLine;
    public int lines = 5;
    public int charPerLine = 16;
    public float fadeIntensity = 5;
    public Color successColor;
    public Color errorColor;
    public Color warnColor;
    public Color infoColor;
    
    Text[] console;
    string[] msgs;
    Vector3[] colors;
    int rollingInd = 0;

    XRController xr;


    void Start()
    {
        xr = GetComponent<XRController>();
        
        ROSConnection.GetOrCreateInstance().Subscribe<RosConsoleMessage>("unity_console", MessageRecived);
        console = new Text[lines];
        msgs = new string[lines];
        colors = new Vector3[lines];
        console[0] = firstLine;
        for (int i = 1; i < lines; i++)
        {
            console[i] = UnityEngine.Object.Instantiate(firstLine) as Text;
            console[i].transform.SetParent(canvas.transform);
            //console[i].rectTransform.position += new Vector3(0, firstLine.rectTransform.rect.height, 0);
            console[i].rectTransform.anchoredPosition = firstLine.rectTransform.anchoredPosition + new Vector2(0, i*firstLine.rectTransform.rect.height);
            console[i].rectTransform.localPosition = new Vector3(console[i].rectTransform.localPosition.x, console[i].rectTransform.localPosition.y, firstLine.rectTransform.localPosition.z);
            console[i].rectTransform.localScale = new Vector3(1, 1, 1);
            console[i].color = new Color(console[i].color.r, console[i].color.g, console[i].color.b, console[i].color.a-((fadeIntensity * i))/100);
        }
    }

    void MessageRecived(RosConsoleMessage msg)
    {
        Debug.Log("[" + msg.code + "]: " + msg.content);
        DateTime d = DateTimeOffset.FromUnixTimeSeconds(msg.header.stamp.sec).DateTime;
        string cd = d.ToString("HH:mm:ss");
        Vector3 clr;
        switch (msg.code)
        {
            case 0:
                clr = new Vector3(successColor.r, successColor.g, successColor.b);
                    break;
            case 1:
                xr.SendHapticImpulse(0.7f, 2f);
                clr = new Vector3(errorColor.r, errorColor.g, errorColor.b);
                break;
            case 2:
                clr = new Vector3(warnColor.r, warnColor.g, warnColor.b);
                break;
            default:
                clr = new Vector3(infoColor.r, infoColor.g, infoColor.b);
                break;
        }
        
        string str = "[" + cd + "] " + msg.content;
        
        while (str.Length > 0)
        {
            if (str.Length <= charPerLine)
            {
                msgs[rollingInd] = str;
                colors[rollingInd] = clr;
                rollingInd++;
                rollingInd %= lines;
                break;
            } else
            {
                msgs[rollingInd] = str.Substring(0, charPerLine);
                colors[rollingInd] = clr;
                rollingInd++;
                rollingInd %= lines;
                str = str.Substring(charPerLine);
            }
        }

        for (int i = 0; i < lines; i++)
        {
            int ind = (rollingInd - lines - i -1)%lines;
            ind = ind < 0 ? ind + lines : ind;
            console[i].text = msgs[ind];
            console[i].color = new Color(colors[ind].x, colors[ind].y, colors[ind].z, console[i].color.a);
        }
    }
}
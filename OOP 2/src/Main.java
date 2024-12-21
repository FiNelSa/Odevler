public class Main {
    public static void main(String[] args) {
        PC myPC             = new PC        ("RTX 3050", 8, "RYZEN 5 5500", 3600, 19, "B450-A", 16, 3600, 4, 512, 4500);
        SCREEN myScreen     = new SCREEN    ("Excalibur", "VA", 200, 1, 24);
        Keyboard myKeyboard = new Keyboard  ("Razer", "Low-Profile", "Wired", 3);
        Mouse myMouse       = new Mouse     ("SteelSeries", "Wired", 18000, 1000, 5);
        Headset myHeadset   = new Headset   ("Rampage", "Wired", true);
        MousePad myPad      = new MousePad  (90, 40, 3);
    }
}
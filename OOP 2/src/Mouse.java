public class Mouse {
    String brand;
    String wire;
    int DPI;
    int HZ;
    int button;

    public Mouse(String brand, String wire, int DPI, int HZ, int Button){
        this.brand      = brand;
        this.wire       = wire;
        this.DPI        = DPI;
        this.button     = Button;
    }
    public Mouse(){}

    public String getBrand() {
        return brand;
    }
    public void setBrand(String brand) {
        this.brand = brand;
    }

    public String getWire() {
        return wire;
    }
    public void setWire(String wire) {
        this.wire = wire;
    }

    public int getDPI() {
        return DPI;
    }
    public void setDPI(int DPI) {
        this.DPI = DPI;
    }

    public int getHZ() {
        return HZ;
    }
    public void setHZ(int HZ) {
        this.HZ = HZ;
    }

    public int getButton() {
        return button;
    }
    public void setButton(int button) {
        this.button = button;
    }
}

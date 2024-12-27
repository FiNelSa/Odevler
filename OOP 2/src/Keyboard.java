public class Keyboard {
    String brand;
    String type;
    String wire;
    int mMediaButtons;

    public Keyboard(String brand, String type, String wire, int mMediaButtons){
        this.brand          = brand;
        this.type           = type;
        this.wire           = wire;
        this.mMediaButtons  = mMediaButtons;
    }
     public Keyboard(){}

    public String getBrand() {
        return brand;
    }
    public void setBrand(String brand) {
        this.brand = brand;
    }

    public String getType() {
        return type;
    }
    public void setType(String type) {
        this.type = type;
    }

    public String getWire() {
        return wire;
    }
    public void setWire(String wire) {
        this.wire = wire;
    }

    public int getmMediaButtons() {
        return mMediaButtons;
    }
    public void setmMediaButtons(int mMediaButtons) {
        this.mMediaButtons = mMediaButtons;
    }
}
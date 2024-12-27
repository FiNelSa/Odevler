public class Headset {
    String brand;
    String wire;
    Boolean mic;

    public Headset(String brand, String wire, Boolean mic){
        this.brand  = brand;
        this.wire   = wire;
        this.mic    = mic;
    }
    public Headset(){}

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

    public Boolean getMic() {
        return mic;
    }
    public void setMic(Boolean mic) {
        this.mic = mic;
    }
}
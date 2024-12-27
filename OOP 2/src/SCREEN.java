public class SCREEN {
    String  brand;
    String  panel;
    int     hz;
    int     ms;
    int     size;

    public SCREEN(String brand, String panel, int hz, int ms, int size){
        this.brand  = brand;
        this.panel  = panel;
        this.hz     = hz;
        this.ms     = ms;
        this.size   = size;
    }
    public SCREEN(){}

    public String getBrand() {
        return brand;
    }
    public void setBrand(String brand) {
        this.brand = brand;
    }

    public String getPanel() {
        return panel;
    }
    public void setPanel(String panel) {
        this.panel = panel;
    }

    public int getHz() {
        return hz;
    }
    public void setHz(int hz) {
        this.hz = hz;
    }

    public int getMs() {
        return ms;
    }
    public void setMs(int ms) {
        this.ms = ms;
    }

    public int getSize() {
        return size;
    }
    public void setSize(int size) {
        this.size = size;
    }
}
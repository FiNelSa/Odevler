public class MousePad {
    int lenght;
    int width;
    int thickness;

    public MousePad(int Lenght, int Width, int Thickness){
        this.lenght     = Lenght;
        this.width      = Width;
        this.thickness  = Thickness;
    }
    public MousePad(){}

    public int getLenght() {
        return lenght;
    }
    public void setLenght(int lenght) {
        this.lenght = lenght;
    }

    public int getWidth() {
        return width;
    }
    public void setWidth(int width) {
        this.width = width;
    }

    public int getThickness() {
        return thickness;
    }
    public void setThickness(int thickness) {
        this.thickness = thickness;
    }
}
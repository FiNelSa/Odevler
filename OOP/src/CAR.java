public class CAR {
    String brand;
    String model;
    String color;
    int year;
    int hp;
    int cc;

    public CAR(String brand, String model, String color, int year, int hp, int cc){
        this.brand = brand.toUpperCase();
        this.model = model.toUpperCase();
        this.color = color.toUpperCase();
        this.year = year;
        this.hp = hp;
        this.cc = cc;
    }

    public String getBrand() {
        return brand;
    }
    public void setBrand(String brand){
        this.brand = brand.toUpperCase();
    }

    public String getModel() {
        return model;
    }
    public void setModel(String model){
        this.model = model.toUpperCase();
    }

    public String getColor() {
        return color;
    }
    public void setColor(String color){
        this.color = color.toUpperCase();
    }

    public int getYear() {
        return year;
    }
    public void setYear(int year){
        this.year = year;
    }

    public int getHp() {
        return hp;
    }
    public void setHp(int hp){
        this.hp = hp;
    }

    public int getCc() {
        return cc;
    }
    public void setCc(int cc){
        this.cc = cc;
    }
}

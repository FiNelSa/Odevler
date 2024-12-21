public class PC {
    String GPU;
    int GPUMemory;
    String CPU;
    int CPUSpeed;
    int CPUMemory;
    String MotherBoard;
    int RamMemory;
    int RamSpeed;
    int RamDDR;
    int Storage;
    int StorageSpeed;

    public PC(String GPU, int GPUMemory, String CPU, int CPUSpeed, int CPUMemory, String Motherboard, int RamMemory, int RamSpeed, int RamDDR, int Storage, int StorageSpeed){
        this.GPU            = GPU;
        this.GPUMemory      = GPUMemory;
        this.CPU            = CPU;
        this.CPUSpeed       = CPUSpeed;
        this.CPUMemory      = CPUMemory;
        this.MotherBoard    = Motherboard;
        this.RamMemory      = RamMemory;
        this.RamSpeed       = RamSpeed;
        this.RamDDR         = RamDDR;
        this.Storage        = Storage;
        this.StorageSpeed   = StorageSpeed;
    }
    public PC(){}

    public String getGPU() {
        return GPU.toUpperCase();
    }
    public void setGPU(String GPU) {
        this.GPU = GPU.toUpperCase();
    }

    public int getGPUMemory() {
        return GPUMemory;
    }
    public void setGPUMemory(int GPUMemory) {
        this.GPUMemory = GPUMemory;
    }

    public String getCPU() {
        return CPU.toUpperCase();
    }
    public void setCPU(String CPU) {
        this.CPU = CPU.toUpperCase();
    }

    public int getCPUSpeed() {
        return CPUSpeed;
    }
    public void setCPUSpeed(int CPUSpeed) {
        this.CPUSpeed = CPUSpeed;
    }

    public int getCPUMemory() {
        return CPUMemory;
    }
    public void setCPUMemory(int CPUMemory) {
        this.CPUMemory = CPUMemory;
    }

    public String getMotherBoard() {
        return MotherBoard.toUpperCase();
    }
    public void setMotherBoard(String motherBoard) {
        MotherBoard = motherBoard.toUpperCase();
    }

    public int getRamMemory() {
        return RamMemory;
    }
    public void setRamMemory(int ramMemory) {
        RamMemory = ramMemory;
    }

    public int getRamSpeed() {
        return RamSpeed;
    }
    public void setRamSpeed(int ramSpeed) {
        RamSpeed = ramSpeed;
    }

    public int getRamDDR() {
        return RamDDR;
    }
    public void setRamDDR(int ramDDR) {
        RamDDR = ramDDR;
    }

    public int getStorage() {
        return Storage;
    }
    public void setStorage(int storage) {
        Storage = storage;
    }

    public int getStorageSpeed() {
        return StorageSpeed;
    }
    public void setStorageSpeed(int storageSpeed) {
        StorageSpeed = storageSpeed;
    }
}

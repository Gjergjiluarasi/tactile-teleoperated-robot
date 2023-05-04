class AppApi{
    constructor(serviceAddress){
        this.serviceAddress = serviceAddress;
    }
    makeURL(operation){
        const resource = operation;
        return new URL(resource, this.serviceAddress);
    }
    connect(){
        fetch(this.makeURL("connect"))
    }
}

export default AppApi;
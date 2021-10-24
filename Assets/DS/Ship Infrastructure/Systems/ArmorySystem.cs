namespace DeepSpace
{
    public class ArmorySystem : ShipSystem
    {
        private SystemElements<IShooter> shooters;
        public ArmorySystem ()
        {
            this.shooters = new SystemElements<IShooter>();
        }
        public override bool AddModule(Module module)
        {
            bool res = false;
            res = res || shooters.Add(module);
            return res;
        }

        public override bool RemoveModule(Module module)
        {
            bool res = false;
            res = res || shooters.Remove(module);
            return res;
        }
    }

    public interface IShooter
    {
        public bool Shoot(int actionGroup);
    }
}
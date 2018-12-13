{ stdenv
, ros_comm
, mkRosPackage
, python3Packages
, pkgs ? import <nixpkgs> {}
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "chemistry_services";
  version = "master";

  src = ./.;

  propagatedBuildInputs = with python3Packages;
  [ ros_comm web3 multihash voluptuous ipfsapi pkgs.robonomics_comm ];

  meta = with stdenv.lib; {
    description = "Chemistry Services package";
    homepage = http://github.com/vourhey/chemistry-quality-control;
    license = licenses.bsd3;
    maintainers = [ maintainers.vourhey ];
  };
}

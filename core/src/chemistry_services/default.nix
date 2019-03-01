{ stdenv
, mkRosPackage
, robonomics_comm
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "chemistry_services";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [ robonomics_comm ];

  meta = with stdenv.lib; {
    description = "Chemistry Services package";
    homepage = http://github.com/vourhey/chemistry-quality-control;
    license = licenses.bsd3;
    maintainers = with maintainers; [ vourhey ];
  };


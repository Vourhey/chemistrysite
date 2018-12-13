{ nixpkgs ? import ./fetchNixpkgs.nix { } }:

rec {
  chemistry_services = nixpkgs.callPackage ./default.nix { };
}

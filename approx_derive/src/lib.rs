extern crate proc_macro;

use proc_macro2::TokenStream;
use quote::quote;
use syn::Type;
use std::mem::discriminant;

#[proc_macro_derive(Approx)]
pub fn approx_derive(
    tokens: proc_macro::TokenStream,
) -> proc_macro::TokenStream {
    let ast = syn::parse(tokens).unwrap();

    impl_approx(&ast)
}

fn impl_approx(ast: &syn::DeriveInput) -> proc_macro::TokenStream {
    let name = &ast.ident;
    let fields = match &ast.data {
        syn::Data::Struct(d) => d.fields.clone(),
        _ => panic!("Approx derive macro only supports structs"),
    };

    let mut epsilon_type: Option<Type> = Option::None;
    for field in &fields {
        match epsilon_type {
            None => epsilon_type = Some(field.ty.clone()),
            Some(ref eps_type) if (discriminant(eps_type) == discriminant(&field.ty)) => continue,
            _ => panic!("multiple different types in the same struct"),
        }
    }

    let epsilon_type = epsilon_type.expect("Struct contains no types");
    

    let (abs_diff, rel_eq, ulps_eq) = match &fields {
        syn::Fields::Named(fields) => {
            let names = fields.named.iter().map(|n| n.ident.as_ref().unwrap());
            let abs_diff: Vec<TokenStream> = names
                .clone()
                .map(|name| quote! { self.#name.abs_diff_eq(&other.#name, epsilon) })
                .collect();
            let rel_eq: Vec<TokenStream> = names
                .clone()
                .map(|name| quote! { self.#name.relative_eq(&other.#name, epsilon, max_relative)})
                .collect();
            let ulps_eq: Vec<TokenStream> = names
                .clone()
                .map(|name| quote! { self.#name.ulps_eq(&other.#name, epsilon, max_ulps)})
                .collect();

            (abs_diff, rel_eq, ulps_eq)
        }
        syn::Fields::Unnamed(fields) => {
            let names = fields
                .unnamed
                .iter()
                .enumerate()
                .map(|(i, _)| syn::Index::from(i));
            let abs_diff: Vec<TokenStream> = names
                .clone()
                .map(|name| quote! { self.#name.abs_diff_eq(other.#name, epsilon) })
                .collect();
            let rel_eq: Vec<TokenStream> = names
                .clone()
                .map(|name| quote! { self.#name.relative_eq(&other.#name, epsilon, max_relative) })
                .collect();
            let ulps_eq: Vec<TokenStream> = names
                .clone()
                .map(|name| quote! { self.#name.ulps_eq(&other.#name, epsilon, max_ulps) })
                .collect();

            (abs_diff, rel_eq, ulps_eq)
        }
        syn::Fields::Unit => todo!(),
    };

    let gen = quote! {
        impl AbsDiffEq for #name {
            type Epsilon = <#epsilon_type as AbsDiffEq>::Epsilon;

            fn default_epsilon() -> Self::Epsilon {
                #epsilon_type::default_epsilon()
            }

            fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
                #(#abs_diff) && *
            }
        }

        impl RelativeEq for #name {
            fn default_max_relative() -> Self::Epsilon {
                #epsilon_type::default_max_relative()
            }

            fn relative_eq(
                &self,
                other: &Self,
                epsilon: Self::Epsilon,
                max_relative: Self::Epsilon,
            ) -> bool {
                #(#rel_eq) && *
            }
        }

        impl UlpsEq for #name {
            fn default_max_ulps() -> u32 {
                #epsilon_type::default_max_ulps()
            }

            fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
                #(#ulps_eq) && *
            }
        }
    };
    gen.into()
}

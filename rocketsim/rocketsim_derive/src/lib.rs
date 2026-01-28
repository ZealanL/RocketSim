use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemStruct};

/// Macro for implementing fast-hashing of a data struct with `zerocopy` and `derive_fast_hash`
///
/// *(I added this because I don't like doing things by-hand because I am lazy)*
#[proc_macro_attribute]
pub fn fast_hash_struct(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let input = parse_macro_input!(item as ItemStruct);
    let name = &input.ident;

    let expanded = quote! {
        #[repr(packed)]
        #[derive(zerocopy::Immutable, zerocopy::IntoBytes)]
        #input
        derive_hash_fast::derive_hash_fast_zerocopy!(#name);
    };
    TokenStream::from(expanded)
}
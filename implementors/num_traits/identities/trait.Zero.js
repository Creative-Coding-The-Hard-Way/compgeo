(function() {var implementors = {};
implementors["nalgebra"] = [{"text":"impl&lt;T, R:&nbsp;<a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimName.html\" title=\"trait nalgebra::base::dimension::DimName\">DimName</a>, C:&nbsp;<a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimName.html\" title=\"trait nalgebra::base::dimension::DimName\">DimName</a>&gt; <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"type\" href=\"nalgebra/base/type.OMatrix.html\" title=\"type nalgebra::base::OMatrix\">OMatrix</a>&lt;T, R, C&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;T: <a class=\"trait\" href=\"nalgebra/base/trait.Scalar.html\" title=\"trait nalgebra::base::Scalar\">Scalar</a> + <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> + <a class=\"trait\" href=\"nalgebra/trait.ClosedAdd.html\" title=\"trait nalgebra::ClosedAdd\">ClosedAdd</a>,<br>&nbsp;&nbsp;&nbsp;&nbsp;<a class=\"struct\" href=\"nalgebra/base/default_allocator/struct.DefaultAllocator.html\" title=\"struct nalgebra::base::default_allocator::DefaultAllocator\">DefaultAllocator</a>: <a class=\"trait\" href=\"nalgebra/base/allocator/trait.Allocator.html\" title=\"trait nalgebra::base::allocator::Allocator\">Allocator</a>&lt;T, R, C&gt;,&nbsp;</span>","synthetic":false,"types":["nalgebra::base::alias::OMatrix"]},{"text":"impl&lt;T:&nbsp;<a class=\"trait\" href=\"nalgebra/trait.SimdRealField.html\" title=\"trait nalgebra::SimdRealField\">SimdRealField</a>&gt; <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"nalgebra/geometry/struct.Quaternion.html\" title=\"struct nalgebra::geometry::Quaternion\">Quaternion</a>&lt;T&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;T::<a class=\"associatedtype\" href=\"nalgebra/trait.SimdValue.html#associatedtype.Element\" title=\"type nalgebra::SimdValue::Element\">Element</a>: <a class=\"trait\" href=\"nalgebra/trait.SimdRealField.html\" title=\"trait nalgebra::SimdRealField\">SimdRealField</a>,&nbsp;</span>","synthetic":false,"types":["nalgebra::geometry::quaternion::Quaternion"]},{"text":"impl&lt;T:&nbsp;<a class=\"trait\" href=\"nalgebra/trait.SimdRealField.html\" title=\"trait nalgebra::SimdRealField\">SimdRealField</a>&gt; <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"nalgebra/geometry/struct.DualQuaternion.html\" title=\"struct nalgebra::geometry::DualQuaternion\">DualQuaternion</a>&lt;T&gt; <span class=\"where fmt-newline\">where<br>&nbsp;&nbsp;&nbsp;&nbsp;T::<a class=\"associatedtype\" href=\"nalgebra/trait.SimdValue.html#associatedtype.Element\" title=\"type nalgebra::SimdValue::Element\">Element</a>: <a class=\"trait\" href=\"nalgebra/trait.SimdRealField.html\" title=\"trait nalgebra::SimdRealField\">SimdRealField</a>,&nbsp;</span>","synthetic":false,"types":["nalgebra::geometry::dual_quaternion::DualQuaternion"]}];
implementors["num_complex"] = [{"text":"impl&lt;T:&nbsp;<a class=\"trait\" href=\"https://doc.rust-lang.org/1.61.0/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a> + <a class=\"trait\" href=\"num_traits/trait.Num.html\" title=\"trait num_traits::Num\">Num</a>&gt; <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"num_complex/struct.Complex.html\" title=\"struct num_complex::Complex\">Complex</a>&lt;T&gt;","synthetic":false,"types":["num_complex::Complex"]}];
implementors["num_rational"] = [{"text":"impl&lt;T:&nbsp;<a class=\"trait\" href=\"https://doc.rust-lang.org/1.61.0/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a> + <a class=\"trait\" href=\"num_integer/trait.Integer.html\" title=\"trait num_integer::Integer\">Integer</a>&gt; <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"num_rational/struct.Ratio.html\" title=\"struct num_rational::Ratio\">Ratio</a>&lt;T&gt;","synthetic":false,"types":["num_rational::Ratio"]}];
implementors["num_traits"] = [];
implementors["simba"] = [{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.f32.html\">f32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.f32.html\">f32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.f32.html\">f32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.f32.html\">f32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 16]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.f64.html\">f64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.f64.html\">f64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.f64.html\">f64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i128.html\">i128</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 1]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i128.html\">i128</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i128.html\">i128</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i16.html\">i16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i16.html\">i16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i16.html\">i16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i16.html\">i16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 16]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i16.html\">i16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 32]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i32.html\">i32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i32.html\">i32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i32.html\">i32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i32.html\">i32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 16]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i64.html\">i64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i64.html\">i64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i64.html\">i64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i8.html\">i8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i8.html\">i8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i8.html\">i8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i8.html\">i8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 16]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.i8.html\">i8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 32]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.isize.html\">isize</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.isize.html\">isize</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.isize.html\">isize</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u128.html\">u128</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 1]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u128.html\">u128</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u128.html\">u128</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u16.html\">u16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u16.html\">u16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u16.html\">u16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u16.html\">u16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 16]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u16.html\">u16</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 32]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u32.html\">u32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u32.html\">u32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u32.html\">u32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u32.html\">u32</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 16]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u64.html\">u64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u64.html\">u64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u64.html\">u64</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u8.html\">u8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u8.html\">u8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u8.html\">u8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u8.html\">u8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 16]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.u8.html\">u8</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 32]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.usize.html\">usize</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 2]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.usize.html\">usize</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 4]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.AutoSimd.html\" title=\"struct simba::simd::AutoSimd\">AutoSimd</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">[</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.usize.html\">usize</a><a class=\"primitive\" href=\"https://doc.rust-lang.org/1.61.0/std/primitive.array.html\">; 8]</a>&gt;","synthetic":false,"types":["simba::simd::auto_simd_impl::AutoSimd"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.WideF32x4.html\" title=\"struct simba::simd::WideF32x4\">WideF32x4</a>","synthetic":false,"types":["simba::simd::wide_simd_impl::WideF32x4"]},{"text":"impl <a class=\"trait\" href=\"num_traits/identities/trait.Zero.html\" title=\"trait num_traits::identities::Zero\">Zero</a> for <a class=\"struct\" href=\"simba/simd/struct.WideF32x8.html\" title=\"struct simba::simd::WideF32x8\">WideF32x8</a>","synthetic":false,"types":["simba::simd::wide_simd_impl::WideF32x8"]}];
if (window.register_implementors) {window.register_implementors(implementors);} else {window.pending_implementors = implementors;}})()
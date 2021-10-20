using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

namespace HutongGames.PlayMaker.Actions
{

	[ActionCategory("Post Processing Stack V2")]
	[Tooltip("Modify Color Grading During Runtime.")]
	public class PostProcessingColorGrading : FsmStateAction
	{
		#region public variables
				
		[RequiredField]
		[ObjectType(typeof(PostProcessProfile))]
		[Tooltip("Post Processing Profile to modify")]
		public FsmObject ObjectProfile;

		[Tooltip("Activate or deactivate Bloom Layer")]
		public FsmBool ActiveLayer;
		[Tooltip("Activate or deactivate Bloom Effect")]
		public FsmBool ActiveEffect;

		[Tooltip("Activate or deactivate Mode")]
		public FsmBool ModeActive;
		[Tooltip("Set the Color Grading Mode")]
		public GradingMode ColorGradingType;

		[Tooltip("Activate or deactivate Mode")]
		public FsmBool ToneMappingActive;
		[Tooltip("Set the Tone Mapping Mode")]
		public Tonemapper ToneMappingType;

		[Tooltip("Activate or deactivate LookUp Texture")]
		public FsmBool LookupTextureActive;
		[Tooltip("Set the Lookup Texture")]
		public FsmTexture LookupTexture;

		[Tooltip("Activate Temperature modification")]
		public FsmBool TemperatureActive;
		[HasFloatSlider(-100f, 100f)]
		[Tooltip("Intensity of Temperature")]
		public FsmFloat Temperature;

		[Tooltip("Activate Tint modification")]
		public FsmBool TintActive;
		[HasFloatSlider(-100f, 100f)]
		[Tooltip("Tint Intensity")]
		public FsmFloat Tint;

		[Tooltip("Activate Post Exposition")]
		public FsmBool PostExposureActive;
		[HasFloatSlider(-20f, 20f)]
		[Tooltip("Post Exposure coef")]
		public FsmFloat PostExposureEv;

		[Tooltip("Activate Color modification")]
		public FsmBool ColorActive;
		[Tooltip("Set the Color Filter")]
		public FsmColor ColorFilter;

		[Tooltip("Activate Hue Shift")]
		public FsmBool HueShiftActive;
		[HasFloatSlider(-180f, 180f)]
		[Tooltip("Set Hue Shift Value")]
		public FsmFloat HueShift;

		[Tooltip("Activate Saturation")]
		public FsmBool SaturationActive;
		[HasFloatSlider(-100f, 100f)]
		[Tooltip("Set Saturation value")]
		public FsmFloat Saturation;

		[Tooltip("Activate Contrast")]
		public FsmBool ContrastActive;
		[HasFloatSlider(-100f, 100f)]
		[Tooltip("set Contrast value")]
		public FsmFloat Contrast;

		[Tooltip("Activate Red Channel")]
		public FsmBool RedChannelActive;
		[HasFloatSlider(-200f, 200f)]
		[Tooltip("set Contrast value")]
		public FsmFloat RedInRedValue;
		public FsmFloat GreenInRedValue;
		public FsmFloat BlueInRedValue;
		[Tooltip("Activate Green Channel")]
		public FsmBool GreenChannelActive;
		[HasFloatSlider(-200f, 200f)]
		[Tooltip("set Contrast value")]
		public FsmFloat RedInGreenValue;
		public FsmFloat GreenInGreenValue;
		public FsmFloat BlueInGreenValue;
		[Tooltip("Activate Blue Channnel")]
		public FsmBool BlueChannelActive;
		[HasFloatSlider(-200f, 200f)]
		[Tooltip("set Contrast value")]
		public FsmFloat RedInBlueValue;
		public FsmFloat GreenInBlueValue;
		public FsmFloat BlueInBlueValue;

		[Tooltip("Activate Lift")]
		public FsmBool LiftActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("set Lift value")]
		public FsmFloat LiftR;
		[HasFloatSlider(0f, 1f)]
		public FsmFloat LiftG;
		[HasFloatSlider(0f, 1f)]
		public FsmFloat LiftB;
		[HasFloatSlider(-1f, 1f)]
		public FsmFloat LiftW;

		[Tooltip("Activate Gamma")]
		public FsmBool GammaActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("set Lift value")]
		public FsmFloat GammaR;
		[HasFloatSlider(0f, 1f)]
		public FsmFloat GammaG;
		[HasFloatSlider(0f, 1f)]
		public FsmFloat GammaB;
		[HasFloatSlider(-1f, 1f)]
		public FsmFloat GammaW;

		[Tooltip("Activate Gamma")]
		public FsmBool GainActive;
		[HasFloatSlider(0f, 1f)]
		[Tooltip("set Lift value")]
		public FsmFloat GainR;
		[HasFloatSlider(0f, 1f)]
		public FsmFloat GainG;
		[HasFloatSlider(0f, 1f)]
		public FsmFloat GainB;
		[HasFloatSlider(-1f, 1f)]
		public FsmFloat GainW;

		[Tooltip("Repeat every frame.")]
		public bool everyFrame;
		#endregion
		
		#region Private variables
		private ColorGrading LutLayer;
		private PostProcessProfile UsedProfile; 		
		#endregion

		// Code that runs on entering the state.
		public override void OnEnter()
		{	
			SetLutParameters();
			if(!everyFrame)
			{
				Finish();
			}
		}

		// Code that runs every frame.
		public override void OnUpdate()
		{
			SetLutParameters();
		}

		void SetLutParameters()
		{
							
				UsedProfile = ObjectProfile.Value as PostProcessProfile;
				UsedProfile.TryGetSettings(out LutLayer);
				if(ObjectProfile != null & UsedProfile.HasSettings<ColorGrading>())
				{
					if(LutLayer!=null)
					{
						LutLayer.active = ActiveLayer.Value; //Active Color Grading Layer
						LutLayer.enabled.value = ActiveEffect.Value; // Active Color Grading Effect

						LutLayer.gradingMode.overrideState = ModeActive.Value;//Activate Mode
						if(ModeActive.Value)
						{ 	
							LutLayer.gradingMode.value = ColorGradingType; // Color Gradding Type
						}
						LutLayer.tonemapper.overrideState = ToneMappingActive.Value;//Activate Tone Mapping
						if(ToneMappingActive.Value)
						{ 	
							LutLayer.tonemapper.value = ToneMappingType; // Tone Mapping
						}
						LutLayer.temperature.overrideState = TemperatureActive.Value;//Activate Temperature
						if(TemperatureActive.Value)
						{ 	
							LutLayer.temperature.value = Temperature.Value; // Temperature
						}
						LutLayer.tint.overrideState = TintActive.Value;//Activate Tint
						if(TintActive.Value)
						{ 	
							LutLayer.tint.value = Tint.Value; // Tint
						}
						LutLayer.postExposure.overrideState = PostExposureActive.Value;//Activate Post Exposure
						if(PostExposureActive.Value)
						{ 	
							LutLayer.postExposure.value = PostExposureEv.Value; // Post Exposure
						}
						LutLayer.colorFilter.overrideState = ColorActive.Value;//Activate Color 
						if(ColorActive.Value)
						{ 	
							LutLayer.colorFilter.value = ColorFilter.Value; // Color Filter
						}
						LutLayer.hueShift.overrideState = HueShiftActive.Value;//Activate Hue Shift
						if(HueShiftActive.Value)
						{ 	
							LutLayer.hueShift.value = HueShift.Value; // Hue Shift
						}
						LutLayer.saturation.overrideState = SaturationActive.Value;//Activate saturation
						if(SaturationActive.Value)
						{ 	
							LutLayer.saturation.value = Saturation.Value; // saturation
						}
						LutLayer.contrast.overrideState = ContrastActive.Value;//Activate Contrast
						if(ContrastActive.Value)
						{ 	
							LutLayer.contrast.value = Contrast.Value; // Contrast
						}

						LutLayer.lift.overrideState = LiftActive.Value;//Activate Lift
						if(LiftActive.Value)
						{ 	
							Vector4 LiftValue = new Vector4(LiftR.Value,LiftG.Value,LiftB.Value,LiftW.Value);
							LutLayer.lift.value = LiftValue; // Lift
						}
						LutLayer.gamma.overrideState = GammaActive.Value;//Activate Gamma
						if(GammaActive.Value)
						{ 	
							Vector4 GammaValue = new Vector4(GammaR.Value,GammaG.Value,GammaB.Value,GammaW.Value);
							LutLayer.gamma.value = GammaValue; // Gamma
						}
						LutLayer.gain.overrideState = GainActive.Value;//Activate Gain
						if(GainActive.Value)
						{ 	
							Vector4 GainValue = new Vector4(GainR.Value,GainG.Value,GainB.Value,GainW.Value);
							LutLayer.gain.value = GainValue; // Gain
						}

						LutLayer.externalLut.overrideState = LookupTextureActive.Value;//Activate External Texture
						if(LookupTextureActive.Value)
						{ 	
							LutLayer.externalLut.value = LookupTexture.Value; // LUT Texture
						}
						LutLayer.brightness.overrideState = PostExposureActive.Value;//Activate Blue Channel
						if(PostExposureActive.Value)
						{ 	
							LutLayer.brightness.value = PostExposureEv.Value; // Blue Channel
						}
						LutLayer.ldrLut.overrideState = LookupTextureActive.Value;//Activate Ldr Texture
						if(LookupTextureActive.Value)
						{ 	
							LutLayer.ldrLut.value = LookupTexture.Value; // Ldr Texture
						}


						LutLayer.redCurve.overrideState = RedChannelActive.Value;//Activate Blue Channel
						if(RedChannelActive.Value)
						{ 	
							LutLayer.mixerRedOutRedIn.value = RedInRedValue.Value; // Blue Channel
							LutLayer.mixerRedOutGreenIn.value = GreenInRedValue.Value; // Blue Channel
							LutLayer.mixerRedOutBlueIn.value = BlueInRedValue.Value; // Blue Channel
						}
						LutLayer.greenCurve.overrideState = GreenChannelActive.Value;//Activate Blue Channel
						if(GreenChannelActive.Value)
						{ 	
							LutLayer.mixerGreenOutRedIn.value = RedInGreenValue.Value; // Blue Channel
							LutLayer.mixerGreenOutGreenIn.value = GreenInGreenValue.Value; // Blue Channel
							LutLayer.mixerGreenOutBlueIn.value = BlueInGreenValue.Value; // Blue Channel
						}
						LutLayer.blueCurve.overrideState = BlueChannelActive.Value;//Activate Blue Channel
						if(BlueChannelActive.Value)
						{ 	
							LutLayer.mixerBlueOutRedIn.value = RedInBlueValue.Value; // Blue Channel
							LutLayer.mixerBlueOutGreenIn.value = GreenInBlueValue.Value; // Blue Channel
							LutLayer.mixerBlueOutBlueIn.value = BlueInBlueValue.Value; // Blue Channel
						}
				}
			}
		}
	}
}


